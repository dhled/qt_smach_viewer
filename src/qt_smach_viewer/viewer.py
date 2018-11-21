#!/usr/bin/env python

# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
# Copyright (c) 2013, Jonathan Bohren, The Johns Hopkins University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#   * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#   * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Jonathan Bohren
# Edit : Ludovic DELVAL 2018

import rospy
import rospkg
import os
import threading
import pickle
import time
import smach
import textwrap

from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from python_qt_binding.QtWidgets import *
from python_qt_binding import loadUi

from qt_smach_viewer.msg import SmachStateMachineStatus,\
                                SmachContainerStructure,\
                                SmachStateMachineStructure
from qt_smach_viewer.xdot import xdot_qt


### Helper Functions
def graph_attr_string(attrs):
    """Generate an xdot graph attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ';\n'.join(attrs_strs)+';\n'

def attr_string(attrs):
    """Generate an xdot node attribute string."""
    attrs_strs = ['"'+str(k)+'"="'+str(v)+'"' for k,v in attrs.iteritems()]
    return ' ['+(', '.join(attrs_strs))+']'

def get_parent_path(path):
    """Get the parent path of an xdot node."""
    path_tokens = path.split('/')
    if len(path_tokens) > 2:
        parent_path = '/'.join(path_tokens[0:-1])
    else:
        parent_path = '/'.join(path_tokens[0:1])
    return parent_path

def get_label(path):
    """Get the label of an xdot node."""
    path_tokens = path.split('/')
    return path_tokens[-1]

def hex2t(color_str):
    """Convert a hexadecimal color strng into a color tuple."""
    color_tuple = [int(color_str[i:i+2],16)/255.0    for i in range(1,len(color_str),2)]
    return color_tuple


class ContainerNode():
    """
    This class represents a given container in a running SMACH system.

    Its primary use is to generate dotcode for a SMACH container. It has
    methods for responding to structure and status messages from a SMACH
    introspection server, as well as methods for updating the styles of a
    graph once it's been drawn.
    """
    def __init__(self, server_name, msg):
        # Store path info
        self._server_name = server_name
        self._path = msg.path
        splitpath = msg.path.split('/')
        self._label = splitpath[-1]
        self._dir = '/'.join(splitpath[0:-1])

        self._children = msg.children
        self._internal_outcomes = msg.internal_outcomes
        self._outcomes_from = msg.outcomes_from
        self._outcomes_to = msg.outcomes_to
        self._initial_states = msg.initial_states
        self._container_outcomes = msg.container_outcomes
        # Status
        self._active_states = []
        self._last_active_states = []
        self._info = ''

    def update_structure(self, msg):
        """Update the structure of this container from a given message. Return True if anything changes."""
        needs_update = False

        if self._children != msg.children\
                or self._internal_outcomes != msg.internal_outcomes\
                or self._outcomes_from != msg.outcomes_from\
                or self._outcomes_to != msg.outcomes_to\
                or self._container_outcomes != msg.container_outcomes\
                or self._initial_states != msg.initial_states:
            needs_update = True

        if needs_update:
            self._children = msg.children
            self._internal_outcomes = msg.internal_outcomes
            self._outcomes_from = msg.outcomes_from
            self._outcomes_to = msg.outcomes_to
            self._initial_states = msg.initial_states
            self._container_outcomes = msg.container_outcomes

        return needs_update

    def update_status(self, msg_active_states):
        """Update the known userdata and active state set and return True if the graph needs to be redrawn."""

        # Initialize the return value
        needs_update = False

        if len(self._active_states) > 0:  # We have active states currently running we check if they are still here
            for state in self._active_states:
                path = self._path + "/" + state
                if(path not in msg_active_states):
                    self._active_states.remove(state)
                    needs_update = True

        # Adding new update state
        for state in msg_active_states:
            path = state[:state.rfind('/')]
            child_state = state[state.rfind('/')+1:]
            if(self._path == path):
                if(child_state not in self._active_states):
                    self._active_states.append(child_state)
                    needs_update = True

        return needs_update

    def get_dotcode(self, selected_paths, closed_paths, depth, max_depth, containers, show_all, label_wrapper, attrs={}):
        """Generate the dotcode representing this container.

        @param selected_paths: The paths to nodes that are selected
        @closed paths: The paths that shouldn't be expanded
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to which we should traverse the tree
        @param containers: A dict of containers keyed by their paths
        @param show_all: True if implicit transitions should be shown
        @param label_wrapper: A text wrapper for wrapping element names
        @param attrs: A dict of dotcode attributes for this cluster
        """

        dotstr = 'subgraph "cluster_%s" {\n' % (self._path)
        if depth == 0:
            attrs['color'] = '#00000000'
            attrs['fillcolor'] = '#0000000F'

        dotstr += graph_attr_string(attrs)

        # Add start/terimate target
        proxy_attrs = {
                'URL': self._path,
                'shape': 'plaintext',
                'color': 'gray',
                'fontsize': '18',
                'fontweight': '18',
                'rank': 'min',
                'height': '0.01'}
        proxy_attrs['label'] = '\\n'.join(label_wrapper.wrap(self._label))
        dotstr += '"%s" %s;\n' % (
                '/'.join([self._path, '__proxy__']),
                attr_string(proxy_attrs))

        # Check if we should expand this container
        if max_depth == -1 or depth <= max_depth:
            # Add container outcomes
            dotstr += 'subgraph "cluster_%s" {\n' % '/'.join([self._path, '__outcomes__'])
            outcomes_attrs = {
                    'style': 'rounded,filled',
                    'rank': 'sink',
                    'color': '#FFFFFFFF',#'#871C34',
                    'fillcolor': '#FFFFFF00'#'#FE464f3F'#'#DB889A'
                    }
            dotstr += graph_attr_string(outcomes_attrs)

            for outcome_label in self._container_outcomes:
                outcome_path = ':'.join([self._path, outcome_label])
                outcome_attrs = {
                        'shape': 'box',
                        'height': '0.3',
                        'style': 'filled,rounded',
                        'fontsize': '12',
                        'fillcolor': '#FE464f',#'#EDC2CC',
                        'color': '#780006',#'#EBAEBB',
                        'fontcolor': '#780006',#'#EBAEBB',
                        'label': '\\n'.join(label_wrapper.wrap(outcome_label)),
                        'URL': ':'.join([self._path, outcome_label])
                        }
                dotstr += '"%s" %s;\n' % (outcome_path, attr_string(outcome_attrs))
            dotstr += "}\n"

            # Iterate over children
            for child_label in self._children:
                child_attrs = {
                        'style': 'filled,setlinewidth(2)',
                        'color': '#000000FF',
                        'fillcolor': '#FFFFFF00'
                        }

                child_path = '/'.join([self._path, child_label])
                # Generate dotcode for children
                if child_path in containers:
                    child_attrs['style'] += ',rounded'

                    dotstr += containers[child_path].get_dotcode(
                            selected_paths,
                            closed_paths,
                            depth+1, max_depth,
                            containers,
                            show_all,
                            label_wrapper,
                            child_attrs)
                else:
                    child_attrs['label'] = '\\n'.join(label_wrapper.wrap(child_label))
                    child_attrs['URL'] = child_path
                    dotstr += '"%s" %s;\n' % (child_path, attr_string(child_attrs))

            # Iterate over edges
            internal_edges = zip(
                    self._internal_outcomes,
                    self._outcomes_from,
                    self._outcomes_to)

            # Add edge from container label to initial state
            internal_edges += [('', '__proxy__', initial_child) for initial_child in self._initial_states]
            has_explicit_transitions = []
            for (outcome_label, from_label, to_label) in internal_edges:
                if to_label != 'None' or outcome_label == to_label:
                    has_explicit_transitions.append(from_label)

            # Draw internal edges
            for (outcome_label, from_label, to_label) in internal_edges:

                from_path = '/'.join([self._path, from_label])

                if show_all \
                        or to_label != 'None'\
                        or from_label not in has_explicit_transitions \
                        or (outcome_label == from_label) \
                        or from_path in containers:
                    # Set the implicit target of this outcome
                    if to_label == 'None':
                        to_label = outcome_label

                    to_path = '/'.join([self._path, to_label])

                    edge_attrs = {
                            'URL': ':'.join([from_path, outcome_label, to_path]),
                            'fontsize': '12',
                            'label': '\\n'.join(label_wrapper.wrap(outcome_label))}
                    edge_attrs['style'] = 'setlinewidth(2)'

                    # Hide implicit
                    # if not show_all and to_label == outcome_label:
                    #    edge_attrs['style'] += ',invis'

                    from_key = '"%s"' % from_path
                    if from_path in containers:
                        if max_depth == -1 or depth+1 <= max_depth:
                            from_key = '"%s:%s"' % (from_path, outcome_label)
                        else:
                            edge_attrs['ltail'] = 'cluster_'+from_path
                            from_path = '/'.join([from_path, '__proxy__'])
                            from_key = '"%s"' % (from_path)

                    to_key = ''
                    if to_label in self._container_outcomes:
                        to_key = '"%s:%s"' % (self._path, to_label)
                        edge_attrs['color'] = '#00000055'  # '#780006'
                    else:
                        if to_path in containers:
                            edge_attrs['lhead'] = 'cluster_'+to_path
                            to_path = '/'.join([to_path, '__proxy__'])
                        to_key = '"%s"' % to_path

                    dotstr += '%s -> %s %s;\n' % (
                            from_key, to_key, attr_string(edge_attrs))

        dotstr += '}\n'
        return dotstr

    def set_styles(self, status, selected_paths, depth, max_depth, items, subgraph_shapes, containers):
        """Update the styles for a list of containers without regenerating the dotcode.

        This function is called recursively to update an entire tree.

        @param selected_paths: A list of paths to nodes that are currently selected.
        @param depth: The depth to start traversing the tree
        @param max_depth: The depth to traverse into the tree
        @param items: A dict of all the graph items, keyed by url
        @param subgraph_shapes: A dictionary of shapes from the rendering engine
        @param containers: A dict of all the containers
        """

        # Color root container
        """
        if depth == 0:
            container_shapes = subgraph_shapes['cluster_'+self._path]
            container_color = (0,0,0,0)
            container_fillcolor = (0,0,0,0)

            for shape in container_shapes:
                shape.pen.color = container_color
                shape.pen.fillcolor = container_fillcolor
                """

        # Color shapes for outcomes
        # Color children
        if max_depth == -1 or depth <= max_depth:
            # Iterate over children
            for child_label in self._children:
                child_path = '/'.join([self._path, child_label])

                child_color = [0.5, 0.5, 0.5, 1]
                child_fillcolor = [1, 1, 1, 1]
                child_linewidth = 2

                if not(status):
                    active_color = hex2t('#FBA80CFF')
                    active_fillcolor = hex2t('#F7F7B2FF')
                else:
                    active_color = hex2t('#5C7600FF')
                    active_fillcolor = hex2t('#C0F700FF')

                initial_color = hex2t('#000000FF')

                if child_label in self._active_states:
                    # Check if the child is active
                    child_color = active_color
                    child_fillcolor = active_fillcolor
                    child_linewidth = 5
                elif child_label in self._initial_states:
                    # Initial style
                    child_color = initial_color
                    child_linewidth = 2

                # Check if the child is selected
                if child_path in selected_paths:
                    child_color = hex2t('#FB000DFF')

                # Generate dotcode for child containers

                if child_path in containers:
                    subgraph_id = 'cluster_'+child_path
                    if subgraph_id in subgraph_shapes:
                        if child_label in self._active_states:
                            child_fillcolor[3] = 0.25
                        elif 0 and child_label in self._initial_states:
                            child_fillcolor[3] = 0.25
                        else:
                            if max_depth > 0:
                                v = 1.0-0.25*((depth+1)/float(max_depth))
                            else:
                                v = 0.85
                            child_fillcolor = [v, v, v, 1.0]

                        for shape in subgraph_shapes['cluster_'+child_path]:
                            pen = shape.pen
                            if len(pen.color) > 3:
                                pen_color_opacity = pen.color[3]
                                if pen_color_opacity < 0.01:
                                    pen_color_opacity = 0
                            else:
                                pen_color_opacity = 0.5
                            shape.pen.color = child_color[0:3]+[pen_color_opacity]
                            shape.pen.fillcolor = [child_fillcolor[i] for i in range(min(3,len(pen.fillcolor)))]
                            shape.pen.linewidth = child_linewidth

                        # Recurse on this child
                        containers[child_path].set_styles(
                                status,
                                selected_paths,
                                depth+1, max_depth,
                                items,
                                subgraph_shapes,
                                containers)
                else:
                    if child_path in items:
                        for shape in items[child_path].shapes:
                            if not isinstance(shape, xdot_qt.TextShape):
                                shape.pen.color = child_color
                                shape.pen.fillcolor = child_fillcolor
                                shape.pen.linewidth = child_linewidth
                    else:
                        # print child_path+" NOT IN "+str(items.keys())
                        pass


class SmachViewerWidget(QWidget):
    """
    This class provides a GUI application for viewing SMACH plans.
    """

    update_ud = pyqtSignal(int)

    def __init__(self, parent):
        QWidget.__init__(self, parent)

        # load the ui
        path = rospkg.RosPack().get_path("qt_smach_viewer")
        path = os.path.join(path, "resources/layouts/ssm_viewer.ui")
        loadUi(path, self)

        # Create graph
        self._containers = {}
        self._top_containers = {}
        self._update_cond = threading.Condition()
        self._needs_refresh = True
        self.dotstr = ''

        # combobox signal
        self.path_cb.activated.connect(self.set_path)

        # depth spinner
        self.depth_sb.valueChanged.connect(self.set_depth)

        # width_spinner
        self.labelwidth_sb.valueChanged.connect(self.set_label_width)

        #
        self.ud_path_cb.activated.connect(self.selection_changed)
        self.update_ud.connect(self.selection_changed)

        # Init Value
        self._show_all_transitions = False
        self._auto_focus = False
        self._max_depth = -1
        self._label_wrapper = textwrap.TextWrapper(40, break_long_words=True)
        self._path = '/'
        self._needs_zoom = True
        self._structure_changed = True
        self._isActive = False

        # Create dot graph widget
        self.widget = xdot_qt.DotWidget(self)
        self.widget.register_select_callback(self.select_cb)
        self.dotgraph_ly.addWidget(self.widget)

        self._containers = {}
        self._selected_paths = []
        self._user_data = smach.UserData()
        # Message subscribers
        self._structure_sub = rospy.Subscriber("/introspection/structure",
                                               SmachStateMachineStructure,
                                               self._structure_msg_update,
                                               queue_size=1,
                                               )
        self._statatus_sub = rospy.Subscriber("/introspection/status",
                                              SmachStateMachineStatus,
                                              self._status_msg_update,
                                              queue_size=1,
                                              )

        # Start a thread in the background to update the server list
        self._keep_running = True

        self._update_graph_thread = threading.Thread(target=self._update_graph)
        self._update_graph_thread.start()

    def mousePressEvent(self, event):
        pass

    def Reset(self):
        self.path_cb.clear()
        self.path_cb.insertItem(0, str("/"))
        self.ud_path_cb.clear()
        self.ud_path_cb.insertItem(0, str("/"))

        self._selected_paths = []

        self._containers = {}
        self._top_containers = {}
        self._needs_refresh = True
        self.dotstr = ''
        rospy.sleep(1.0)
        self._set_path("/")

    def OnQuit(self, event):
        """Quit Event: kill threads and wait for join."""
        with self._update_cond:
            self._keep_running = False
            self._update_cond.notify_all()

        self._update_graph_thread.join()

    def update_graph(self):
        """Notify all that the graph needs to be updated."""
        with self._update_cond:
            self._update_cond.notify_all()

    def on_set_initial_state(self, event):
        """Event: Change the initial state of the server."""
        state_path = self._selected_paths[0]
        parent_path = get_parent_path(state_path)
        state = get_label(state_path)

        server_name = self._containers[parent_path]._server_name
        self._client.set_initial_state(server_name,
                                       parent_path,
                                       [state],
                                       timeout=rospy.Duration(60.0))

    def set_path(self, value):
        """Event: Change the viewable path and update the graph."""
        self._path = self.path_cb.currentText()
        self._needs_zoom = True
        self.update_graph()

    def _set_path(self, path):
        self._path = path
        self._needs_zoom = True
        self.update_graph()

    def set_depth(self, event):
        """Event: Change the maximum depth and update the graph."""
        self._max_depth = self.depth_sb.value()
        self._needs_zoom = True
        self.update_graph()

    def _set_max_depth(self, max_depth):
        self._max_depth = max_depth
        self.depth_spinner.SetValue(max_depth)
        self._needs_zoom = True
        self.update_graph()

    def set_label_width(self, event):
        """Event: Change the label wrapper width and update the graph."""
        self._label_wrapper.width = self.labelwidth_sb.value()
        self._needs_zoom = True
        self.update_graph()

    def toggle_all_transitions(self, event):
        """Event: Change whether automatic transitions are hidden and update the graph."""
        self._show_all_transitions = not self._show_all_transitions
        self._structure_changed = True
        self.update_graph()

    def toggle_auto_focus(self, event):
        """Event: Enable/Disable automatically focusing"""
        self._auto_focus = not self._auto_focus
        self._needs_zoom = self._auto_focus
        self._structure_changed = True
        if not self._auto_focus:
            self._set_path('/')
            self._max_depth(-1)
        self.update_graph()

    def select_cb(self, item, event):
        """Event: Click to select a graph node to display user data and update the graph."""
        # Only set string status
        # Left button-up
            # Store this item's url as the selected path
        self._selected_paths = [item]
        # Update the selection dropdown
        self.ud_path_cb.insertItem(0, str(item))
        self.ud_path_cb.setCurrentIndex(0)
        self.ud_path_cb.activated.emit(0)
        self.update_graph()

    def selection_changed(self, event):
        """Event: Selection dropdown changed."""
        path_input_str = self.ud_path_cb.currentText()
        # Check the path is non-zero length
        if len(path_input_str) > 0:
            # Split the path (state:outcome), and get the state path
            path = path_input_str.split(':')[0]
            # Get the container corresponding to this path, since userdata is
            # stored in the containers
            if path not in self._containers:
                if path == '/':
                    if(len(self._top_containers) > 0):
                        parent_path = self._top_containers.keys()[0]
                    else:
                        parent_path = path
                else:
                    parent_path = get_parent_path(path)
            else:
                parent_path = path

            if parent_path in self._containers:
                # Enable the initial state button for the selection
                for row in range(self.ud_tw.rowCount()):
                    self.ud_tw.removeRow(0)
                # Generate the userdata string
                for (k, v) in self._user_data.iteritems():
                    self.ud_tw.insertRow(self.ud_tw.rowCount())
                    self.ud_tw.setItem(self.ud_tw.rowCount()-1, 0, QTableWidgetItem(str(k)))
                    self.ud_tw.setItem(self.ud_tw.rowCount()-1, 1, QTableWidgetItem(str(v)))

            else:
                # Disable the initial state button for this selection
                #self.is_button.Disable()
                pass
            self.ud_tw.resizeRowsToContents()

    def _structure_msg_update(self, msg):
        """Update the structure of the SMACH plan (re-generate the dotcode)."""
        # Just return if we're shutting down
        if not self._keep_running:
            return
        for container in msg.containers:
        # Get the node path
            path = container.path
            pathsplit = path.split('/')
            parent_path = '/'.join(pathsplit[0:-1])

            rospy.logdebug("RECEIVED: "+path)
            rospy.logdebug("CONTAINERS: "+str(self._containers.keys()))

            # Initialize redraw flag
            needs_redraw = False

            if path in self._containers:
                rospy.logdebug("UPDATING: "+path)

                # Update the structure of this known container
                needs_redraw = self._containers[path].update_structure(container)
            else:
                rospy.logdebug("CONSTRUCTING: "+path)

                # Create a new container
                new_container = ContainerNode("", container)
                self._containers[path] = new_container

                # Store this as a top container if it has no parent
                if parent_path == '':
                    self._top_containers[path] = new_container

                # Append paths to selector
                if(self.path_cb.findText(path) == -1):
                    self.path_cb.addItem(path)
                if(self.ud_path_cb.findText(path) == -1):
                    self.ud_path_cb.addItem(path)

                # We need to redraw thhe graph if this container's parent is already known
                if parent_path in self._containers:
                    needs_redraw = True

            # Update the graph if necessary
            if needs_redraw:
                with self._update_cond:
                    self._structure_changed = True
                    self._needs_zoom = True  # TODO: Make it so you can disable this
                    self._update_cond.notify_all()

    def _status_msg_update(self, msg):
        """Process status messages."""
        # Check if we're in the process of shutting down
        if not self._keep_running:
            return
        self._user_data = pickle.loads(msg.user_data)
        self.update_ud.emit(0)
        if(msg.status == 1):
            self._isActive = True

        if self._auto_focus and len(msg.info) > 0:
            self._set_path(msg.info)
            self._set_max_depth(msg.info.count('/')-1)

        # Get the path to the updating conainer
        # rospy.logdebug("STATUS MSG: "+path)
        for container in self._containers.values():
            # Parse Path
            if container.update_status(msg.active_states):
                with self._update_cond:
                    self._update_cond.notify_all()

    def _update_graph(self):
        """This thread continuously updates the graph when it changes.

        The graph gets updated in one of two ways:

          1: The structure of the SMACH plans has changed, or the display
          settings have been changed. In this case, the dotcode needs to be
          regenerated.

          2: The status of the SMACH plans has changed. In this case, we only
          need to change the styles of the graph.
        """
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                # Wait for the update condition to be triggered
                self._update_cond.wait()

                # Get the containers to update
                containers_to_update = {}
                if self._path in self._containers:
                    # Some non-root path
                    containers_to_update = {self._path:self._containers[self._path]}
                elif self._path == '/':
                    # Root path
                    containers_to_update = self._top_containers

                # Check if we need to re-generate the dotcode (if the structure changed)
                # TODO: needs_zoom is a misnomer
                if self._structure_changed or self._needs_zoom:
                    dotstr = "digraph {\n\t"
                    dotstr += ';'.join([
                        "compound=true",
                        "outputmode=nodesfirst",
                        "labeljust=l",
                        "nodesep=0.5",
                        "minlen=2",
                        "mclimit=5",
                        "clusterrank=local",
                        "ranksep=0.75",
                        # "remincross=true",
                        # "rank=sink",
                        "ordering=\"\"",
                        ])
                    dotstr += ";\n"

                    # Generate the rest of the graph
                    # TODO: Only re-generate dotcode for containers that have changed

                    for path, tc in containers_to_update.iteritems():
                        dotstr += tc.get_dotcode(
                                self._selected_paths, [],
                                0, self._max_depth,
                                self._containers,
                                self._show_all_transitions,
                                self._label_wrapper)
                    else:
                        pass
                        #dotstr += '"__empty__" [label="Path not available.", shape="plaintext"]'

                    dotstr += '\n}\n'
                    self.dotstr = dotstr
                    # Set the dotcode to the new dotcode, reset the flags
                    self.set_dotcode(dotstr, zoom=False)
                    self._structure_changed = False

                # Update the styles for the graph if there are any updates
                for path, tc in containers_to_update.iteritems():
                    tc.set_styles(
                            self._isActive,
                            self._selected_paths,
                            0, self._max_depth,
                            self.widget.items_by_url,
                            self.widget.subgraph_shapes,
                            self._containers)

                # Redraw
                self.widget.update()

    def set_dotcode(self, dotcode, zoom=True):
        """Set the xdot view's dotcode and refresh the display."""
        # Set the new dotcode
        if self.widget.set_dotcode(dotcode, None):
            # Re-zoom if necessary
            if zoom or self._needs_zoom:
                self.widget.zoom_to_fit()
                self._needs_zoom = False
            # Set the refresh flag
            self._needs_refresh = True

    def _update_tree(self):
        """Update the tree view."""
        while self._keep_running and not rospy.is_shutdown():
            with self._update_cond:
                self._update_cond.wait()
                self.tree.DeleteAllItems()
                self._tree_nodes = {}
                for path, tc in self._top_containers.iteritems():
                    self.add_to_tree(path, None)

    def add_to_tree(self, path, parent):
        """Add a path to the tree view."""
        if parent is None:
            container = self.tree.AddRoot(get_label(path))
        else:
            container = self.tree.AppendItem(parent, get_label(path))

        # Add children to tree
        for label in self._containers[path]._children:
            child_path = '/'.join([path, label])
            if child_path in self._containers.keys():
                self.add_to_tree(child_path, container)
            else:
                self.tree.AppendItem(container, label)

    def OnIdle(self, event):
        """Event: On Idle, refresh the display if necessary, then un-set the flag."""
        if self._needs_refresh:
            self.Refresh()
            # Re-populate path combo
            self._needs_refresh = False

    def SaveDotGraph(self, event):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        directory = rospkg.get_ros_home()+'/dotfiles/'
        if not os.path.exists(directory):
                os.makedirs(directory)
        filename = directory+timestr+'.dot'
        print('Writing to file: %s' % filename)
        with open(filename, 'w') as f:
            f.write(self.dotstr)

    def set_filter(self, filter):
        self.widget.set_filter(filter)
