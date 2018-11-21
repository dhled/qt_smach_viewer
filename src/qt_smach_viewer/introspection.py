import rospy
import pickle
import threading
import smach

from qt_smach_viewer.msg import SmachStateMachineStatus, SmachContainerStructure, SmachStateMachineStructure
from std_msgs.msg import Header

STATUS_TOPIC = '/smach/container_status'
INIT_TOPIC = '/smach/container_init'
STRUCTURE_TOPIC = '/smach/container_structure'


class ContainerProxy(object):
    """Smach Container Introspection proxy.

    This class is used as a container for introspection and debugging.
    """
    def __init__(self, container, path, userdata, update_lock):
        """Constructor for tree-wide data structure.
        """
        self.path = path
        self.active_states = []
        self.userdata = userdata
        self.update_lock = update_lock
        # Set transition callback
        container.register_start_cb(self._update_status)
        container.register_transition_cb(self._update_status)
        container.register_termination_cb(self._update_status_termination)

    def _update_status(self, userdata, active_states, outcome=None):
        self.update_lock.acquire()
        self.active_states = []
        for state in active_states:
            self.active_states.append(self.path+'/'+state)
        self.userdata.update(userdata)
        self.update_lock.notify_all()
        self.update_lock.release()

    def _update_status_termination(self, userdata, unactive_states, outcome=None):
        self.update_lock.acquire()
        for unactive_state in unactive_states:
            remove_state = self.path+'/' + unactive_state
            for i_state in range(len(self.active_states)):
                if(self.active_states[i_state] == remove_state):
                    del self.active_states[i_state]
        self.userdata.update(userdata)
        self.update_lock.notify_all()
        self.update_lock.release()


class PublishStatus(threading.Thread):
    def __init__(self, server, lock):
        threading.Thread.__init__(self)
        self.server = server
        self.lock = lock
        self._status_publisher = rospy.Publisher("/introspection/status",
                                                 SmachStateMachineStatus,
                                                 latch=True,
                                                 queue_size=1,
                                                 )

    def _create_message(self):
        # pickle possible userdata
        # TODO optimisation
        copy_dict = {}
        for data, value in self.server.userdata._data.iteritems():
            try:
                pickle.dumps(value, 2)
                copy_dict[data] = value
            except Exception:
                copy_dict[data] = "Not Available"
                continue

        # get all actives states
        actives_states = []
        for container in self.server.proxies:
            actives_states = list(set().union(actives_states, container.active_states))

        msg = SmachStateMachineStatus(Header(stamp=rospy.Time.now()),
                                      1, actives_states, pickle.dumps(copy_dict, 2), "")
        return msg

    def run(self):
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.lock.wait()
            if(self.server.stopped_lock.is_set()):
                self.lock.release()
                return
            if(self._status_publisher.get_num_connections() > 0):
                msg = self._create_message()
                self._status_publisher.publish(msg)
            self.lock.release()


class IntrospectionServer():
    """Server for providing introspection and control for smach."""
    def __init__(self, state_machine):

        # Init EventLock
        self.update_lock = threading.Condition()
        self.stopped_lock = threading.Event()
        # Init Publisher Thread
        self._thread_publish_status = PublishStatus(self, self.update_lock)
        self._structure_publisher = rospy.Publisher("/introspection/structure", SmachStateMachineStructure, latch=True, queue_size=1)
        # Containers
        self._state_machine = state_machine
        self.proxies = []
        self._structure = SmachStateMachineStructure()
        # UserData
        self.userdata = smach.UserData()

    def start(self):
        # Construct proxies
        self.construct(self._state_machine)
        self._structure_publisher.publish(self._structure)
        self._thread_publish_status.start()

    def stop(self):
        self.stopped_lock.set()
        self.update_lock.acquire()
        self.update_lock.notifyAll()
        self.update_lock.release()

    def _construct_structure(self, state, path):
        children = list(state.get_children().keys())

        internal_outcomes = []
        outcomes_from = []
        outcomes_to = []
        for (outcome, from_label, to_label) in state.get_internal_edges():
            internal_outcomes.append(str(outcome))
            outcomes_from.append(str(from_label))
            outcomes_to.append(str(to_label))
        container_outcomes = state.get_registered_outcomes()

        # Construct structure message
        structure_msg = SmachContainerStructure(
                path,
                children,
                state.get_initial_states(),
                internal_outcomes,
                outcomes_from,
                outcomes_to,
                container_outcomes,
                state.get_registered_input_keys(),
                state.get_registered_output_keys())
        return structure_msg

    def construct(self, state, path="root"):
        """Recursively construct proxies to containers and structure for the state machine."""
        # Construct a new proxy
        proxy = ContainerProxy(state, path, self.userdata, self.update_lock)
        self._structure.containers.append(self._construct_structure(state, path))
        # Get a list of children that are also containers
        for (label, child) in state.get_children().iteritems():
            # If this is also a container, recurse into it
            if isinstance(child, smach.container.Container):
                self.construct(child, path+'/'+label)
        # Store the proxy
        self.proxies.append(proxy)

    def clear(self):
        """Clear all proxies in this server."""
        self.proxies = []
