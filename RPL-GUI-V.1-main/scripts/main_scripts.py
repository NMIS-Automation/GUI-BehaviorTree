import sys
import json
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QVBoxLayout, QWidget,
                             QHBoxLayout, QDialog, QDialogButtonBox, QFormLayout, QComboBox, QLineEdit, QPushButton, QToolButton, QStyle, QListWidget, QListWidgetItem, QAbstractItemView, QMessageBox, QMenu, QAction, QFileDialog, QCheckBox)
from PyQt5.QtGui import QPixmap, QPainter, QPen, QDrag, QPolygonF, QIcon, QBrush, QColor
from PyQt5.QtCore import Qt, QMimeData, QPointF, QLineF, QRect, QPoint, QSize
import subprocess
import shlex
import time
import os
import math
from main_gui import Ui_MainWindow  # Ensure this import matches the file name

ASYNC_COLOR = Qt.green
SYNC_COLOR = Qt.blue


class HistoryWidget(QWidget):
    def __init__(self, parent=None):
        super(HistoryWidget, self).__init__(parent)
        self.parent_widget = parent  # Store the reference to the parent widget
        self.setWindowTitle("History Behavior Tree")
        self.setFixedSize(300, 400)
        self.layout = QVBoxLayout(self)

        self.history_list = QListWidget()
        self.history_list.setSelectionMode(QAbstractItemView.MultiSelection)  # Enable multiple selection
        self.history_list.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        self.layout.addWidget(self.history_list)

        # Add a layout for the top buttons
        self.top_layout = QHBoxLayout()
        self.layout.addLayout(self.top_layout)

        # Add the three-line button
        self.menu_button = QPushButton("≡", self)
        self.menu_button.setFixedSize(30, 30)
        self.menu_button.clicked.connect(self.show_menu)
        self.top_layout.addWidget(self.menu_button)

        # Add the Select button
        self.select_button = QPushButton("Select", self)
        self.select_button.setFixedSize(70, 30)
        self.select_button.clicked.connect(self.select_behavior_tree)  # Connect the Select button
        self.top_layout.addWidget(self.select_button)

        # Add the theme toggle checkbox
        self.themeToggleButton = QCheckBox("Dark Theme")
        self.themeToggleButton.stateChanged.connect(self.parent_widget.toggle_theme)
        self.top_layout.addWidget(self.themeToggleButton)

        self.top_layout.addStretch()

        self.saved_behavior_trees = []  # List to store saved behavior trees

    def show_menu(self):
        menu = QMenu(self)

        save_modify_action = QAction("Save and Modify", self)
        save_compound_action = QAction("Save as Compound", self)

        save_modify_action.triggered.connect(self.parent_widget.save_and_modify)
        save_compound_action.triggered.connect(self.parent_widget.save_as_compound)

        menu.addAction(save_modify_action)
        menu.addAction(save_compound_action)

        menu.exec_(self.menu_button.mapToGlobal(self.menu_button.rect().bottomLeft()))

    def add_item(self, name, pixmap, behavior_tree):
        item = QListWidgetItem(name)
        item.setData(Qt.UserRole, pixmap)
        item.setData(Qt.UserRole + 1, behavior_tree)  # Store the behavior tree in the item

        # Create a label to show the pixmap
        label = QLabel()
        label.setPixmap(pixmap)
        label.setScaledContents(True)
        label.setFixedSize(280, 280)  # Adjust as needed

        # Add the label to the list item
        widget = QWidget()
        layout = QVBoxLayout(widget)
        layout.addWidget(label)
        layout.addStretch()

        item.setSizeHint(widget.sizeHint())
        self.history_list.addItem(item)
        self.history_list.setItemWidget(item, widget)

    def select_behavior_tree(self):
        selected_items = self.history_list.selectedItems()
        if not selected_items:
            QMessageBox.warning(self, "Selection Error", "Please select one or more behavior trees from the history.")
            return

        combined_behavior_tree = {}
        combined_connections = []
        offset_y = 0

        for selected_item in selected_items:
            behavior_tree = selected_item.data(Qt.UserRole + 1)
            combined_behavior_tree, combined_connections = self.combine_trees(combined_behavior_tree, combined_connections, behavior_tree, offset_y)
            offset_y += 200

        print("Combined Behavior Tree:", combined_behavior_tree)
        print("Combined Connections:", combined_connections)
        self.parent_widget.load_behavior_tree({'shapes': combined_behavior_tree, 'connections': combined_connections})

    def combine_trees(self, tree1, connections1, tree2, offset_y):
        combined_tree = tree1.copy()
        combined_connections = connections1.copy()
        id_mapping = {}

        for shape_id, details in tree2['shapes'].items():
            new_shape_id = f"{shape_id}_{len(combined_tree)}"
            new_position = QPointF(details['position'][0], details['position'][1] + offset_y)
            combined_tree[new_shape_id] = {
                'type': details['type'],
                'position': (new_position.x(), new_position.y()),
                'children': details['children'],
                'name': details['name'],
                'task': details['task'],
                'desc': details['desc'],
                'color': details.get('color', QColor(SYNC_COLOR).name()),
                'manual_color': details.get('manual_color', None)
            }
            id_mapping[shape_id] = new_shape_id
            print(f"Added shape {new_shape_id} at position {new_position}")

        for connection in tree2['connections']:
            start_id = connection['start_id']
            end_id = connection['end_id']
            if start_id in id_mapping and end_id in id_mapping:
                new_start_id = id_mapping[start_id]
                new_end_id = id_mapping[end_id]
                combined_connections.append({
                    'start_id': new_start_id,
                    'end_id': new_end_id,
                    'type': connection['type']
                })
                print(f"Connected {new_start_id} to {new_end_id} with type {connection['type']}")

        return combined_tree, combined_connections

    def get_parent_id(self, shape_id, behavior_tree):
        for parent_id, details in behavior_tree.items():
            for child in details['children']:
                if child['id'] == shape_id:
                    return parent_id
        return None

    def get_siblings(self, parent_id, behavior_tree):
        if parent_id is None:
            return []
        return [child['id'] for child in behavior_tree[parent_id]['children']]

    def get_child_conditions(self, parent_id, connections, behavior_tree):
        if parent_id is None:
            return {}

        child_conditions = {}
        for child in behavior_tree[parent_id]['children']:
            child_id = child['id']
            for connection in connections:
                if connection['start_id'] == parent_id and connection['end_id'] == child_id:
                    condition = connection['type']
                    if condition in child_conditions:
                        child_conditions[condition].append(child_id)
                    else:
                        child_conditions[condition] = [child_id]
        return child_conditions


class ShapeButton(QToolButton):
    def __init__(self, shape, parent=None):
        super(ShapeButton, self).__init__(parent)
        self.shape = shape
        self.setFixedSize(100, 100)  # Increase the button size to accommodate larger icons
        self.setIconSize(QSize(80, 80))  # Set the icon size to be larger

        if shape == "Circle":
            pixmap = QPixmap(80, 80)  # Increase the pixmap size for a larger icon
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)  # Enable antialiasing for smoother edges
            painter.setPen(QPen(Qt.black, 3))  # Thicker pen for visibility
            painter.setBrush(QBrush(Qt.gray, Qt.SolidPattern))  # Fill the circle with gray color
            painter.drawEllipse(10, 10, 60, 60)  # Draw a larger circle
            painter.end()
            self.setIcon(QIcon(pixmap))

        elif shape == "Square":
            pixmap = QPixmap(80, 80)  # Increase the pixmap size for a larger icon
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setPen(QPen(Qt.black, 3))
            painter.setBrush(QBrush(Qt.gray, Qt.SolidPattern))  # Ensure the square is filled with a color
            painter.drawRect(10, 10, 60, 60)  # Draw a larger square
            painter.end()
            self.setIcon(QIcon(pixmap))

        elif shape == "Eraser":
            pixmap = QPixmap(80, 80)  # Increase the pixmap size for a larger icon
            pixmap.fill(Qt.transparent)
            painter = QPainter(pixmap)
            painter.setRenderHint(QPainter.Antialiasing)
            painter.setPen(QPen(Qt.black, 3))
            painter.setBrush(QBrush(Qt.red, Qt.SolidPattern))
            painter.drawRect(20, 30, 40, 20)  # Draw a larger rectangle representing an eraser
            painter.end()
            self.setIcon(QIcon(pixmap))

        self.setText(shape)


    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            mimeData = QMimeData()
            mimeData.setText(self.shape)

            drag = QDrag(self)
            drag.setMimeData(mimeData)

            pixmap = QPixmap(self.size())
            self.render(pixmap)
            drag.setPixmap(pixmap)
            drag.setHotSpot(event.pos())

            drag.exec_(Qt.MoveAction)


class PopupDialog(QDialog):
    def __init__(self, parent=None, popup_type='link'):
        super(PopupDialog, self).__init__(parent)
        self.setWindowTitle("Shape/Link Info")
        self.setFixedSize(300, 250)  # Increased size to accommodate the new color option

        layout = QFormLayout(self)

        if popup_type == 'shape':
            self.nameLabel = QLabel("ROBOT NAME")
            self.nameLineEdit = QLineEdit()
            layout.addRow(self.nameLabel, self.nameLineEdit)

            self.taskLabel = QLabel("TASK")
            self.taskLineEdit = QLineEdit()
            layout.addRow(self.taskLabel, self.taskLineEdit)

            self.descLabel = QLabel("TASK DESCRIPTION")
            self.descLineEdit = QLineEdit()
            layout.addRow(self.descLabel, self.descLineEdit)

            # Add the color selection combo box
            self.colorLabel = QLabel("COLOR")
            self.colorComboBox = QComboBox()
            self.colorComboBox.addItems(["DEFAULT", "BLUE - SYNC", "GREEN - ASYNC"])  # Added a DEFAULT option to reset to original logic
            layout.addRow(self.colorLabel, self.colorComboBox)

        else:
            self.typeLabel = QLabel("TYPE")
            self.typeComboBox = QComboBox()
            self.typeComboBox.addItems(["true", "false", "yes", "no"])
            layout.addRow(self.typeLabel, self.typeComboBox)

        self.buttonBox = QDialogButtonBox(QDialogButtonBox.Save | QDialogButtonBox.Cancel)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        layout.addRow(self.buttonBox)

    def get_type(self):
        return self.typeComboBox.currentText() if hasattr(self, 'typeComboBox') else None

    def get_name(self):
        return self.nameLineEdit.text() if hasattr(self, 'nameLineEdit') else None

    def get_task(self):
        return self.taskLineEdit.text() if hasattr(self, 'taskLineEdit') else None

    def get_desc(self):
        return self.descLineEdit.text() if hasattr(self, 'descLineEdit') else None

    def get_color(self):
        return self.colorComboBox.currentText() if hasattr(self, 'colorComboBox') else None


class DropArea(QLabel):
    def __init__(self, parent=None):
        super(DropArea, self).__init__(parent)
        self.setAcceptDrops(True)
        self.setAlignment(Qt.AlignCenter)
        self.setStyleSheet("QLabel { background-color : white; }")
        self.shapes = []
        self.connections = []
        self.last_right_clicked = None
        self.behavior_tree = {}
        self.dragging_shape = None
        self.offset = QPointF()
        self.double_click_timer = None
        self.scale_factor = 1.0
        self.setMouseTracking(True)
        self.panning = False
        self.pan_start = QPoint()
        self.translation = QPointF(0, 0)

    def get_parent_id(self, shape_id):
        for parent_id, details in self.behavior_tree.items():
            for child in details['children']:
                if child['id'] == shape_id:
                    return parent_id
        return None

    def get_siblings(self, parent_id):
        if parent_id is None:
            return []
        return [child['id'] for child in self.behavior_tree[parent_id]['children']]

    def get_child_conditions(self, parent_id):
        if parent_id is None:
            return {}

        child_conditions = {}
        for child in self.behavior_tree[parent_id]['children']:
            child_id = child['id']
            for connection in self.connections:
                start_id = self.get_shape_id_at_pos(connection[0])
                end_id = self.get_shape_id_at_pos(connection[1])
                if start_id == parent_id and end_id == child_id:
                    condition = connection[2]
                    if condition in child_conditions:
                        child_conditions[condition].append(child_id)
                    else:
                        child_conditions[condition] = [child_id]
        return child_conditions

    def dragEnterEvent(self, event):
        if event.mimeData().hasText():
            event.acceptProposedAction()
        else:
            event.ignore()

    def dropEvent(self, event):
        position = (event.pos() - self.translation) / self.scale_factor
        shape = event.mimeData().text()

        print(f"Dropped shape: {shape} at position: {position}")

        if shape == "Eraser":
            self.erase_shape(position)
            self.erase_connection(position)
        else:
            self.shapes.append((shape, position))
            shape_id = f"{shape}_{len(self.shapes)}"
            self.show_popup('shape')
            if self.popup_name is not None:
                self.behavior_tree[shape_id] = {
                    'type': shape,
                    'position': (position.x(), position.y()),
                    'children': [],
                    'name': self.popup_name,
                    'task': self.popup_task,
                    'desc': self.popup_desc,
                    'color': QColor(SYNC_COLOR).name(),  # Initialize with default color
                    'manual_color': None  # No manual color initially
                }
                print(f"Added shape {shape_id} at position {position}")
            self.update()
            event.acceptProposedAction()

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            clicked_pos = (event.pos() - self.translation) / self.scale_factor
            clicked_shape = self.get_shape_at_pos(clicked_pos)
            if clicked_shape:
                if self.last_right_clicked is None:
                    self.last_right_clicked = clicked_shape
                else:
                    start_shape, start_pos = self.last_right_clicked
                    end_shape, end_pos = clicked_shape
                    self.show_popup('link')
                    if self.popup_type:
                        self.connections.append((start_pos, end_pos, self.popup_type))
                        start_shape_id = self.get_shape_id(start_shape, start_pos)
                        end_shape_id = self.get_shape_id(end_shape, end_pos)
                        if start_shape_id and end_shape_id:
                            self.behavior_tree[start_shape_id]['children'].append({
                                'id': end_shape_id,
                                'type': self.popup_type
                            })
                    self.last_right_clicked = None
                    self.update()
        elif event.button() == Qt.LeftButton:
            clicked_pos = (event.pos() - self.translation) / self.scale_factor
            clicked_shape = self.get_shape_at_pos(clicked_pos)
            if clicked_shape:
                self.dragging_shape = clicked_shape
                shape, position = clicked_shape
                self.offset = position - clicked_pos
            else:
                self.double_click_timer = self.startTimer(200)
        elif event.button() == Qt.MiddleButton:
            self.setCursor(Qt.ClosedHandCursor)
            self.pan_start = event.pos()
            self.panning = True

    def mouseMoveEvent(self, event):
        if self.panning:
            delta = event.pos() - self.pan_start
            self.pan_start = event.pos()
            self.translation += delta
            self.update()
        elif self.dragging_shape:
            new_pos = (event.pos() - self.translation) / self.scale_factor + self.offset
            shape, old_pos = self.dragging_shape
            self.update_shape_position(shape, old_pos, new_pos)
            self.dragging_shape = (shape, new_pos)
            self.update()  # Ensure this line is present to continuously redraw the arrows
        else:
            super(DropArea, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.dragging_shape:
                self.dragging_shape = None
        elif event.button() == Qt.MiddleButton:
            self.setCursor(Qt.ArrowCursor)
            self.panning = False

    def mouseDoubleClickEvent(self, event):
        if self.double_click_timer:
            self.killTimer(self.double_click_timer)
            self.double_click_timer = None
        clicked_pos = (event.pos() - self.translation) / self.scale_factor
        clicked_shape = self.get_shape_at_pos(clicked_pos)
        if clicked_shape:
            self.edit_shape(clicked_shape)

    def edit_shape(self, shape_info):
        shape, position = shape_info
        shape_id = self.get_shape_id(shape, position)
        if shape_id:
            details = self.behavior_tree[shape_id]
            self.show_popup('shape', details)
            if self.popup_name is not None:
                details['name'] = self.popup_name
                details['task'] = self.popup_task
                details['desc'] = self.popup_desc

                # Update the color based on the user's selection
                selected_color = self.popup_color  # Get the selected color from the dialog
                if selected_color == "BLUE - SYNC":
                    details['manual_color'] = QColor(SYNC_COLOR).name()
                elif selected_color == "GREEN - ASYNC":
                    details['manual_color'] = QColor(ASYNC_COLOR).name()
                elif selected_color == "DEFAULT":
                    details.pop('manual_color', None)  # Remove manual color to revert to sync/async logic

                print(f"Edited shape {shape_id}: manual_color set to {details.get('manual_color', 'None')}")
                self.update()


    def show_popup(self, popup_type='link', current_details=None):
        dialog = PopupDialog(self, popup_type)
        if current_details:
            if popup_type == 'shape':
                dialog.nameLineEdit.setText(current_details.get('name', ''))
                dialog.taskLineEdit.setText(current_details.get('task', ''))
                dialog.descLineEdit.setText(current_details.get('desc', ''))
                current_color = current_details.get('manual_color', 'DEFAULT')
                dialog.colorComboBox.setCurrentText("BLUE - SYNC" if current_color == QColor(SYNC_COLOR).name() else "GREEN - ASYNC" if current_color == QColor(ASYNC_COLOR).name() else "DEFAULT")
        if dialog.exec_() == QDialog.Accepted:
            self.popup_type = dialog.get_type()
            self.popup_name = dialog.get_name() if popup_type == 'shape' else None
            self.popup_task = dialog.get_task() if popup_type == 'shape' else None
            self.popup_desc = dialog.get_desc() if popup_type == 'shape' else None
            self.popup_color = dialog.get_color() if popup_type == 'shape' else None
        else:
            self.popup_type = None
            self.popup_name = None
            self.popup_task = None
            self.popup_desc = None
            self.popup_color = None


    def timerEvent(self, event):
        self.killTimer(event.timerId())
        self.double_click_timer = None

    def wheelEvent(self, event):
        cursor_pos = event.pos()
        old_scale_factor = self.scale_factor
        if event.angleDelta().y() > 0:
            self.scale_factor *= 1.1
        else:
            self.scale_factor /= 1.1
        scale_ratio = self.scale_factor / old_scale_factor

        # Adjust translation to keep the cursor at the same relative position
        self.translation = cursor_pos - (cursor_pos - self.translation) * scale_ratio

        self.update()

    def get_shape_at_pos(self, pos):
        for shape, position in self.shapes:
            if shape == "Circle" and (position - pos).manhattanLength() < 25:
                return (shape, position)
            elif shape == "Square" and abs(position.x() - pos.x()) < 25 and abs(position.y() - pos.y()) < 25:
                return (shape, position)
            elif shape == "Diamond" and self.is_point_in_diamond(pos, position):
                return (shape, position)
        return None

    def is_point_in_diamond(self, point, center):
        diamond_size = 20  # Half size of the diamond's bounding box
        dx = abs(point.x() - center.x())
        dy = abs(point.y() - center.y())
        return (dx + dy) <= diamond_size

    def update_shape_position(self, shape, old_position, new_position):
        for i, (s, pos) in enumerate(self.shapes):
            if s == shape and pos == old_position:
                self.shapes[i] = (shape, new_position)
                shape_id = self.get_shape_id(shape, old_position)
                if shape_id:
                    self.behavior_tree[shape_id]['position'] = (new_position.x(), new_position.y())
                break

    def get_child_conditions(self, parent_id):
        if parent_id is None:
            return {}

        child_conditions = {}
        for child in self.behavior_tree[parent_id]['children']:
            child_id = child['id']
            for connection in self.connections:
                start_id = self.get_shape_id_at_pos(connection[0])
                end_id = self.get_shape_id_at_pos(connection[1])
                if start_id == parent_id and end_id == child_id:
                    condition = connection[2]
                    if condition in child_conditions:
                        child_conditions[condition].append(child_id)
                    else:
                        child_conditions[condition] = [child_id]
        return child_conditions

    def paintEvent(self, event):
        super(DropArea, self).paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setRenderHint(QPainter.SmoothPixmapTransform)
        painter.translate(self.translation)
        painter.scale(self.scale_factor, self.scale_factor)

        # Draw the connections first
        for connection in self.connections:
            start, end, conn_type = connection
            start_shape = self.get_shape_at_pos(start)
            end_shape = self.get_shape_at_pos(end)
            if start_shape and end_shape:
                self.draw_arrow(painter, start, end, start_shape[0], end_shape[0])
                self.draw_text(painter, start, end, conn_type)

        # Draw the shapes and text labels
        for shape, position in self.shapes:
            shape_id = self.get_shape_id(shape, position)
            if shape_id:
                details = self.behavior_tree.get(shape_id, {})
                manual_color = details.get('manual_color', None)

                if manual_color:
                    brush = QBrush(QColor(manual_color))  # Use manual color if set
                else:
                    parent_id = self.get_parent_id(shape_id)
                    siblings = self.get_siblings(parent_id)
                    child_conditions = self.get_child_conditions(parent_id)

                    if parent_id is None or len(siblings) <= 1:
                        brush = QBrush(QColor(SYNC_COLOR))  # Default to sync color
                    else:
                        is_async = False
                        for condition, child_ids in child_conditions.items():
                            if shape_id in child_ids and len(child_ids) > 1:
                                is_async = True
                                break

                        brush = QBrush(QColor(ASYNC_COLOR)) if is_async else QBrush(QColor(SYNC_COLOR))
                painter.setBrush(brush)

            if shape == "Circle":
                painter.drawEllipse(position, 20, 20)
            elif shape == "Square":
                painter.drawRect(position.x() - 20, position.y() - 20, 40, 40)
            elif shape == "Diamond":
                points = [QPointF(position.x(), position.y() - 20), QPointF(position.x() + 20, position.y()),
                        QPointF(position.x(), position.y() + 20), QPointF(position.x() - 20, position.y())]
                painter.drawPolygon(QPolygonF(points))

            # Draw the text labels next to the node
            name = details.get('name', '')
            task = details.get('task', '')
            if name or task:
                label_text = f"{name}: {task}"
                text_position = QPointF(position.x() + 25, position.y() - 25)  # Adjust the text position as needed
                painter.setPen(Qt.black)
                painter.drawText(text_position, label_text)

        painter.end()



    def draw_arrow(self, painter, start_pos, end_pos, start_shape, end_shape):
        # Calculate the line
        line = QLineF(start_pos, end_pos)

        # Calculate the radius of the node (assumed to be 20, adjust if needed)
        node_radius = 20

        # Calculate the intersection point at the border of the target node
        line_length = line.length()
        if line_length > 0:
            dx = (end_pos.x() - start_pos.x()) / line_length
            dy = (end_pos.y() - start_pos.y()) / line_length
            end_pos_adjusted = QPointF(end_pos.x() - dx * node_radius, end_pos.y() - dy * node_radius)
        else:
            end_pos_adjusted = end_pos

        # Draw the adjusted line
        painter.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        adjusted_line = QLineF(start_pos, end_pos_adjusted)
        painter.drawLine(adjusted_line)

        # Draw a small circle at the adjusted end position
        circle_radius = 5
        painter.setBrush(Qt.black)
        painter.drawEllipse(end_pos_adjusted, circle_radius, circle_radius)

        # Calculate the angle for the arrowhead
        arrow_size = 10
        angle = math.atan2(-adjusted_line.dy(), adjusted_line.dx())

        arrow_p1 = end_pos_adjusted + QPointF(math.sin(angle + math.pi / 3) * arrow_size,
                                            math.cos(angle + math.pi / 3) * arrow_size)
        arrow_p2 = end_pos_adjusted + QPointF(math.sin(angle + math.pi - math.pi / 3) * arrow_size,
                                            math.cos(angle + math.pi - math.pi / 3) * arrow_size)

        arrow_head = [end_pos_adjusted, arrow_p1, arrow_p2]
        painter.setBrush(Qt.black)
        painter.drawPolygon(QPolygonF(arrow_head))




    def draw_text(self, painter, start_pos, end_pos, text):
        middle_point = (start_pos + end_pos) / 2
        painter.drawText(middle_point, text)

    def get_shape_id(self, shape, position):
        for shape_id, details in self.behavior_tree.items():
            if details['type'] == shape and details['position'] == (position.x(), position.y()):
                return shape_id
        return None

    def erase_shape(self, pos):
        for shape, position in self.shapes:
            if (shape == "Circle" and (position - pos).manhattanLength() < 25) or \
                    (shape == "Square" and abs(position.x() - pos.x()) < 25 and abs(position.y() - pos.y()) < 25) or \
                    (shape == "Diamond" and self.is_point_in_diamond(pos, position)):
                self.shapes.remove((shape, position))
                shape_id = self.get_shape_id(shape, position)
                if shape_id:
                    del self.behavior_tree[shape_id]
                    self.connections = [conn for conn in self.connections if conn[0] != position and conn[1] != position]
                self.update()
                break

    def erase_connection(self, pos):
        for connection in self.connections:
            start, end, conn_type = connection
            line = QLineF(start, end)
            if line.p1().x() <= pos.x() <= line.p2().x() or line.p2().x() <= pos.x() <= line.p1().x():
                if line.p1().y() <= pos.y() <= line.p2().y() or line.p2().y() <= pos.y() <= line.p1().y():
                    self.connections.remove(connection)
                    self.update()
                    break

    def get_shape_id_at_pos(self, pos):
        for shape, position in self.shapes:
            if self.get_shape_at_pos(pos) == (shape, position):
                return self.get_shape_id(shape, position)
        return None


class RPLLineGUI(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(RPLLineGUI, self).__init__()
        self.setupUi(self)

        # Initialize ROS core
        self.roscore = None
        try:
            self.roscore = subprocess.Popen(shlex.split('roscore'))
            time.sleep(5)  # Wait for roscore to start
        except subprocess.CalledProcessError as e:
            print("Error starting roscore:", e)

        # Connect buttons to methods
        self.loadView.clicked.connect(self.load_new_view)
        self.saveView.clicked.connect(self.save_view_config)
        self.newSensorView.clicked.connect(self.new_sensor_view)
        self.refreshSensors.clicked.connect(self.refresh_sensors)
        self.logPose.clicked.connect(self.log_pose)
        self.setReference.clicked.connect(self.set_reference)
        self.editPoses.clicked.connect(self.edit_poses)
        self.runButton.clicked.connect(self.test_motion)
        self.saveAllMotions.clicked.connect(self.save_motions)
        self.loadButton.clicked.connect(self.load_motion)
        self.addMotionBttn.clicked.connect(self.add_motion)
        self.deleteMotion.clicked.connect(self.delete_motion)

        # Add logo to the top left corner
        self.logoLabel = QLabel(self)
        self.logoPixmap = QPixmap("/home/nmis/Downloads/nmis-logo-full-colour-e1657196524147.png")  # Replace with the path to your logo
        self.logoLabel.setPixmap(self.logoPixmap)
        self.logoLabel.setAlignment(Qt.AlignCenter)

        # Adjust logo size
        logo_size = self.logoPixmap.size()
        self.logoLabel.setFixedSize(logo_size)

        # Add drag-and-drop shapes to the sidebar
        self.circleButton = ShapeButton("Circle", self)
        self.squareButton = ShapeButton("Square", self)
        self.eraserButton = ShapeButton("Eraser", self)

        # Set larger sizes for the buttons
        self.circleButton.setFixedSize(80, 80)  # Increased size to 80x80
        self.squareButton.setFixedSize(80, 80)  # Increased size to 80x80
        self.eraserButton.setFixedSize(80, 80)  # Increased size to 80x80

        self.shapeLayout = QVBoxLayout()
        self.shapeLayout.addWidget(self.circleButton)
        self.shapeLayout.addWidget(self.squareButton)
        self.shapeLayout.addWidget(self.eraserButton)

        self.shapeContainer = QWidget()
        self.shapeContainer.setLayout(self.shapeLayout)

        self.sidebarLayout = QVBoxLayout()
        self.sidebarLayout.setContentsMargins(0, 0, 0, 0)  # Set all margins to 0 to remove any gap
        self.sidebarLayout.addWidget(self.logoLabel)  # Add logo to the sidebar
        self.sidebarLayout.addWidget(self.shapeContainer)
        self.sidebarLayout.addStretch()  # Add stretch to push the save button to the bottom


        # Add Save button
        self.saveButton = QPushButton("Save Behavior Tree", self)
        self.saveButton.clicked.connect(self.save_behavior_tree)
        self.sidebarLayout.addWidget(self.saveButton)

        # Add Open button
        self.openButton = QPushButton("Open Behavior Tree", self)
        self.openButton.clicked.connect(self.open_behavior_tree)
        self.sidebarLayout.addWidget(self.openButton)

        self.sidebar = QWidget()
        self.sidebar.setLayout(self.sidebarLayout)

        # Initialize DropArea
        self.dropArea = DropArea(self)

        # Initialize History Widget
        self.historyWidget = HistoryWidget(self)

        # Main layout
        self.mainLayout = QHBoxLayout()
        self.mainLayout.addWidget(self.sidebar)
        self.mainLayout.addWidget(self.dropArea)
        self.mainLayout.addWidget(self.historyWidget)

        # Create a central widget and set the main layout
        central_widget = QWidget()
        central_widget.setLayout(self.mainLayout)
        self.setCentralWidget(central_widget)

        # Apply the initial theme
        self.apply_light_theme()

    def apply_dark_theme(self):
        self.load_qss("dark.qss")
        self.dropArea.setStyleSheet("QLabel { background-color : #4B4B4B; }")  # Set to a grey color

    def apply_light_theme(self):
        self.load_qss("light.qss")
        self.dropArea.setStyleSheet("QLabel { background-color : white; }")

    def load_qss(self, file_name):
        try:
            qss_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), file_name)
            with open(qss_file_path, "r") as f:
                self.setStyleSheet(f.read())
        except FileNotFoundError:
            print(f"QSS file '{file_name}' not found.")
        except Exception as e:
            print(f"Error loading QSS file '{file_name}': {e}")

    def toggle_theme(self, state):
        if state == Qt.Checked:
            self.apply_dark_theme()
        else:
            self.apply_light_theme()

    def load_new_view(self):
        pass

    def save_view_config(self):
        pass

    def new_sensor_view(self):
        pass

    def refresh_sensors(self):
        pass

    def log_pose(self):
        pass

    def set_reference(self):
        pass

    def edit_poses(self):
        pass

    def test_motion(self):
        pass

    def save_motions(self):
        pass

    def load_motion(self):
        pass

    def add_motion(self):
        pass

    def delete_motion(self):
        pass

    def save_behavior_tree(self):
        print("Save button clicked")
        behavior_tree = self.dropArea.behavior_tree
        connections = self.dropArea.connections

        if not behavior_tree:
            print("No behavior tree to save")
            return

        behavior_dict = {
            'shapes': {},
            'connections': []
        }
        for shape_id, details in behavior_tree.items():
            behavior_dict['shapes'][shape_id] = {
                'type': details['type'],
                'position': details['position'],
                'children': details.get('children', []),
                'name': details.get('name', ''),
                'task': details.get('task', ''),
                'desc': details.get('desc', ''),
                'color': details.get('color', QColor(SYNC_COLOR).name()),  # Ensure color information is included
                'manual_color': details.get('manual_color', None)  # Include manual color if set
            }
            print(f"Saved shape {shape_id} with color {behavior_dict['shapes'][shape_id]['color']}")

        for connection in connections:
            start, end, conn_type = connection
            start_id = self.dropArea.get_shape_id_at_pos(start)
            end_id = self.dropArea.get_shape_id_at_pos(end)
            if start_id and end_id:
                behavior_dict['connections'].append({
                    'start_id': start_id,
                    'end_id': end_id,
                    'type': conn_type
                })
                print(f"Saved connection from {start_id} to {end_id} of type {conn_type}")

        print("Behavior dict to save:", behavior_dict)  # Debug print

        with open('behavior_tree.json', 'w') as json_file, open('behavior_tree_history.json', 'a') as history_file:
            json.dump(behavior_dict, json_file, indent=4)
            history_file.write(json.dumps(behavior_dict) + "\n")
        print("Behavior tree saved to behavior_tree.json and appended to history")

        pixmap = QPixmap(self.dropArea.size())
        self.dropArea.render(pixmap)
        self.historyWidget.add_item("Behavior Tree", pixmap, behavior_dict)

    def save_and_modify(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Behavior Tree", "", "JSON Files (*.json);;All Files (*)", options=options)
        if file_name:
            behavior_tree = self.dropArea.behavior_tree

            if not behavior_tree:
                print("No behavior tree to save")
                return

            behavior_dict = {
                'shapes': {},
                'connections': []
            }
            for shape_id, details in behavior_tree.items():
                behavior_dict['shapes'][shape_id] = {
                    'type': details['type'],
                    'position': details['position'],
                    'children': details['children'],
                    'name': details['name'],
                    'task': details['task'],
                    'desc': details['desc'],
                    'color': details.get('color', QColor(SYNC_COLOR).name())  # Include color information
                }

            for connection in self.dropArea.connections:
                start_id = self.dropArea.get_shape_id_at_pos(connection[0])
                end_id = self.dropArea.get_shape_id_at_pos(connection[1])
                if start_id and end_id:
                    behavior_dict['connections'].append({
                        'start_id': start_id,
                        'end_id': end_id,
                        'type': connection[2]
                    })

            with open(file_name, 'w') as json_file:
                json.dump(behavior_dict, json_file, indent=4)
            print(f"Behavior tree saved to {file_name}")

    def save_as_compound(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getSaveFileName(self, "Save Behavior Tree as Compound", "", "JSON Files (*.json);;All Files (*)", options=options)
        if file_name:
            print(f"Saving compound behavior tree to: {file_name}")

            behavior_tree = self.dropArea.behavior_tree

            if not behavior_tree:
                print("No behavior tree to save")
                return

            combined_details = {
                'type': 'Diamond',
                'position': (self.dropArea.width() / 2, self.dropArea.height() / 2),
                'children': [],
                'name': 'Compound Node',
                'task': 'Combined Task',
                'desc': 'This is a combined node of all tasks'
            }

            for shape_id, details in behavior_tree.items():
                combined_details['children'].append({
                    'id': shape_id,
                    'type': 'child'
                })

            with open(file_name, 'w') as json_file:
                json.dump({'compound_node': combined_details}, json_file, indent=4)
            print(f"Behavior tree saved to {file_name}")
        else:
            print("Save as Compound operation was canceled")

    def open_behavior_tree(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Open Behavior Tree", "", "JSON Files (*.json);;All Files (*)", options=options)
        if file_name:
            print(f"Loading behavior tree from: {file_name}")
            with open(file_name, 'r') as json_file:
                behavior_dict = json.load(json_file)
                print(f"Behavior tree loaded: {behavior_dict}")

                if 'shapes' in behavior_dict:
                    self.load_behavior_tree(behavior_dict)
                else:
                    shapes = behavior_dict
                    connections = []
                    behavior_dict = {'shapes': shapes, 'connections': connections}
                    self.load_behavior_tree(behavior_dict)
        else:
            print("Open operation was canceled")

    def load_behavior_tree(self, behavior_dict, connections=[]):
        self.dropArea.shapes.clear()
        self.dropArea.connections.clear()
        self.dropArea.behavior_tree.clear()

        print("Loading shapes and connections into the canvas")
        for shape_id, details in behavior_dict['shapes'].items():
            if 'position' in details:
                position = QPointF(details['position'][0], details['position'][1])
                shape = details['type']
                print(f"Loading shape {shape} at position {position} with color {details.get('color', QColor(SYNC_COLOR).name())}")
                self.dropArea.shapes.append((shape, position))
                self.dropArea.behavior_tree[shape_id] = {
                    'type': shape,
                    'position': (position.x(), position.y()),
                    'children': details.get('children', []),
                    'name': details.get('name', ''),
                    'task': details.get('task', ''),
                    'desc': details.get('desc', ''),
                    'color': details.get('color', QColor(SYNC_COLOR).name()),  # Ensure color is stored in behavior tree
                    'manual_color': details.get('manual_color', None)  # Load manual color if present
                }
            else:
                print(f"Error: 'position' key not found in shape details for {shape_id}")

        if not connections:
            for connection in behavior_dict['connections']:
                start_id = connection['start_id']
                end_id = connection['end_id']
                start = QPointF(*behavior_dict['shapes'][start_id]['position'])
                end = QPointF(*behavior_dict['shapes'][end_id]['position'])
                conn_type = connection['type']
                print(f"Connecting {start} to {end} with type {conn_type}")
                self.dropArea.connections.append((start, end, conn_type))
        else:
            self.dropArea.connections = connections

        self.dropArea.update()
        print("Finished loading behavior tree")


if __name__ == "__main__":
    print("Starting application...")
    app = QApplication(sys.argv)
    window = RPLLineGUI()
    window.show()
    print("Application running...")
    sys.exit(app.exec_())
