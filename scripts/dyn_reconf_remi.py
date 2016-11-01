#!/usr/bin/env python

import socket
import remi.gui as gui
from remi.server import start
from remi import App
import rospy
from ast import literal_eval
from dynamic_reconfigure import client as drc
from dynamic_reconfigure import find_reconfigure_services

DEFAULT_WIDTH = 800
DEFAULT_HEIGHT = 25

# NAME_L_SIZE = 200
# MIN_L_SIZE = 90
# EDIT1_SIZE = 300
# MAX_L_SIZE = 90
# EDIT2_SIZE = 90
#FIELD_HEIGHT = 10
NAME_L_SIZE = -1
MIN_L_SIZE = -1
EDIT1_SIZE = -1
MAX_L_SIZE = -1
EDIT2_SIZE = -1
FIELD_HEIGHT = -1
SLIDER_SIZE = 300


class MyApp(App):

    def __init__(self, *args):
        super(MyApp, self).__init__(*args)
        # rospy.spin()

    def main(self, name='world'):
        # the arguments are width - height - layoutOrientationHorizontal
        self.dyn_rec_client = None
        self.wid = gui.Widget(layout_orientation=gui.Widget.LAYOUT_VERTICAL)

        self.hor_servers = gui.Widget(
            layout_orientation=gui.Widget.LAYOUT_HORIZONTAL)
        self.bt = gui.Button('Refresh Dynamic Reconfigure Servers list')
        self.bt.set_on_click_listener(self, 'refresh_servers')

        self.hor_servers.add_child(1, self.bt)
        # This makes the button not be left
        self.bt.style['display'] = 'block'
        self.bt.style['margin'] = '10px auto'
        self.bt.style['float'] = 'none'

        self.refresh_servers()

        # returning the root widget
        return self.wid

    def dyn_rec_conf_callback(self, new_config):
        rospy.loginfo("Updating GUI with:\n" + str(new_config))
        for k in new_config:
            if self.gui_set_funcs_for_param.has_key(k):
                for set_func in self.gui_set_funcs_for_param[k]:
                    set_func(new_config[k])

    def on_dropdown_change(self, widget, value):
        # If we had a previous client, disconnect it
        if self.dyn_rec_client is not None:
            self.dyn_rec_client.close()
        # Get new client
        self.dyn_rec_client = drc.Client(value, timeout=10.0)

        # Get a configuration which ensures we'll have the description too
        curr_conf = self.dyn_rec_client.get_configuration()
        self.gui_set_funcs_for_param = {}
        params_list = self.dyn_rec_client.group_description['parameters']
        if params_list is not None:
            table = gui.Table()
            row = gui.TableRow()
            item = gui.TableTitle()
            item.add_child(str(id(item)), 'Param name')
            row.add_child(str(id(item)), item)
            item = gui.TableTitle()
            item.add_child(str(id(item)), 'Min')
            row.add_child(str(id(item)), item)
            item = gui.TableTitle()
            item.add_child(str(id(item)), 'Edit')
            row.add_child(str(id(item)), item)
            item = gui.TableTitle()
            item.add_child(str(id(item)), 'Max')
            row.add_child(str(id(item)), item)
            item = gui.TableTitle()
            item.add_child(str(id(item)), 'Edit2')
            row.add_child(str(id(item)), item)
            table.add_child(str(id(row)), row)

            for idx, param in enumerate(params_list):
                if param['edit_method'] != '':
                    # Enum
                    param_name = param['name']
                    # WTF, really? the full enum dict is actually a string?
                    enum_dict_as_str = param['edit_method']
                    enum_dict = literal_eval(enum_dict_as_str)
                    description = enum_dict['enum_description']
                    current_value = curr_conf[param_name]
                    enums = enum_dict['enum']
                    # there must be at least one enum
                    enum_type = enums[0]['type']
                    # Create dynamically a method to be called
                    method_name, cb_method = self.add_cb_to_class_by_param_name_and_type(
                        param_name,
                        'enum',
                        enum_type)
                    enum_wid, set_funcs = self.create_enum_row(param_name,
                                                               description,
                                                               current_value,
                                                               cb_method,
                                                               enums)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.add_child(idx, enum_wid)
                elif param['type'] == 'int' or param['type'] == 'double':
                    param_name = param['name']
                    range_min = param['min']
                    range_max = param['max']
                    description = param['description']
                    current_value = curr_conf[param_name]
                    if param['type'] == 'int':
                        step = (range_max - range_min) / 100
                    elif param['type'] == 'double':
                        step = (range_max - range_min) / 100.0

                    # Create dynamically a method to be called
                    method_name, cb_method = self.add_cb_to_class_by_param_name_and_type(
                        param_name,
                        'digit')

                    num_wid, set_funcs = self.create_num_row(param_name,
                                                             description,
                                                             range_min,
                                                             range_max,
                                                             current_value,
                                                             cb_method, step)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.add_child(idx, num_wid)
                elif param['type'] == 'str':
                    param_name = param['name']
                    current_value = curr_conf[param_name]
                    description = param['description']
                    # Create dynamically a method to be called
                    method_name, cb_method = self.add_cb_to_class_by_param_name_and_type(
                        param_name,
                        'string')
                    str_wid, set_funcs = self.create_str_row(param_name,
                                                             description,
                                                             current_value,
                                                             cb_method)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.add_child(idx, str_wid)
                elif param['type'] == 'bool':
                    param_name = param['name']
                    current_value = curr_conf[param_name]
                    description = param['description']
                    # Create dynamically a method to be called
                    method_name, cb_method = self.add_cb_to_class_by_param_name_and_type(
                        param_name,
                        'bool')
                    bool_wid, set_funcs = self.create_bool_row(param_name,
                                                               description,
                                                               current_value,
                                                               cb_method)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.add_child(idx, bool_wid)

        self.wid.add_child(2, table)
        # This must be done later on! HACK! (append sets a margin)
        # This makes the table not stick left, but float in the middle
        table.style['margin'] = '10px auto'

        # Once the GUI is setup, setup the callback for new configs
        self.dyn_rec_client.set_config_callback(self.dyn_rec_conf_callback)

    def add_cb_to_class_by_param_name_and_type(self, param_name, param_type, enum_type=None):
        # Create dynamically a method to be called
        def make_method(parameter_name):
            def _method(widget, new_value):
                # print "Cb for param '" + parameter_name + "' called."
                # print "We got value: " + str(new_value)
                if param_type == 'digit' or param_type == 'string':
                    value_to_set = new_value
                elif param_type == 'enum':
                    # Fix the value as per type
                    value = new_value[
                        new_value.find("(") + 1:new_value.find(")")]
                    if enum_type == 'int':
                        value_to_set = int(value)
                    elif enum_type == 'str':
                        value_to_set = value
                    elif enum_type == 'float':
                        value_to_set = float(value)
                elif param_type == 'bool':
                    # Fix the value as its returned as a string and we need a
                    # boolean
                    if new_value == 'true':
                        value_to_set = True
                    elif new_value == 'false':
                        value_to_set = False
                    else:
                        rospy.logerr(
                            "Error parsing bool parameter in dynamic callback of " + parameter_name)
                self.dyn_rec_client.update_configuration(
                    {parameter_name: value_to_set})

            return _method

        method_name = "callback_func_for_" + param_name
        cb_method = make_method(param_name)
        setattr(self, method_name, cb_method)
        return method_name, cb_method

    def refresh_servers(self):
        self.dynamic_reconfigure_servers = find_reconfigure_services()
        rospy.loginfo("Found dynamic reconfigure servers:\n" +
                      str(self.dynamic_reconfigure_servers))
        self.dropdown = gui.DropDown()
        choose_ddi = gui.DropDownItem("Choose server...")
        self.dropdown.add_child(0, choose_ddi)
        for idx, server_name in enumerate(self.dynamic_reconfigure_servers):
            ddi = gui.DropDownItem(server_name)
            self.dropdown.add_child(idx + 1, ddi)

        self.dropdown.set_on_change_listener(self.on_dropdown_change)
        # using ID 2 to update the dropdown
        self.hor_servers.add_child(2, self.dropdown)
        # This makes the dropdown not be left
        self.dropdown.style['display'] = 'block'
        self.dropdown.style['margin'] = '10px auto'
        self.dropdown.style['float'] = 'none'
        self.wid.add_child(1, self.hor_servers)

    def create_num_row(self, param_name, description, range_min, range_max,
                       current_value, callback_cb, step):
        row = gui.TableRow()
        param_name = gui.Label(param_name,
                               width=NAME_L_SIZE,
                               height=FIELD_HEIGHT)
        param_name.attributes['title'] = description
        min_val = gui.Label(str(range_min),
                            width=MIN_L_SIZE,
                            height=FIELD_HEIGHT)
        range_slider = gui.Slider(width=SLIDER_SIZE, height=FIELD_HEIGHT,
                                  defaultValue=current_value, min=range_min,
                                  max=range_max,
                                  step=step)
        range_slider.set_on_change_listener(callback_cb)
        max_val = gui.Label(str(range_max),
                            width=MAX_L_SIZE,
                            height=FIELD_HEIGHT)
        spin_val = gui.SpinBox(width=EDIT2_SIZE, height=FIELD_HEIGHT,
                               defaultValue=current_value,
                               min=range_min, max=range_max, step=step)
        # https://github.com/dddomodossola/remi/issues/49
        # Added 46 as it's dot so we allow floating point values
        spin_val.attributes[
            spin_val.EVENT_ONKEYPRESS] = 'return event.charCode >= 48 && event.charCode <= 57 || event.charCode == 46 || event.charCode == 13'
        spin_val.set_on_change_listener(callback_cb)
        item = gui.TableItem()
        item.add_child(0, param_name)
        row.add_child(0, item)

        item = gui.TableItem()
        item.add_child(1, min_val)
        row.add_child(1, item)
        min_val.style['float'] = 'none'
        min_val.style['text-align'] = 'center'

        item = gui.TableItem()
        item.add_child(2, range_slider)
        row.add_child(2, item)

        item = gui.TableItem()
        item.add_child(3, max_val)
        row.add_child(3, item)
        max_val.style['float'] = 'none'
        max_val.style['text-align'] = 'center'

        item = gui.TableItem()
        item.add_child(4, spin_val)
        row.add_child(4, item)
        spin_val.style['display'] = 'block'
        spin_val.style['margin'] = '10px auto'
        spin_val.style['float'] = 'none'

        return row, [range_slider.set_value, spin_val.set_value]

    def test_cb(self, *args):
        rospy.loginfo("We got a cb with args: " + str(args))
        rospy.loginfo("Which the arg is of type: ")
        if len(args) >= 1:
            rospy.loginfo(str(type(args[0])))
        # self.dyn_rec_client.update_configuration()

    def create_str_row(self, param_name, description,
                       current_value, callback_cb):
        row = gui.TableRow()
        param_name = gui.Label(param_name,
                               width=NAME_L_SIZE,
                               height=FIELD_HEIGHT)
        param_name.attributes['title'] = description
        text_input = gui.TextInput(
            width=SLIDER_SIZE, height=FIELD_HEIGHT, single_line=True)
        text_input.set_text(current_value)

        text_input.set_on_enter_listener(callback_cb)

        # set_button = gui.Button(80, DEFAULT_HEIGHT, text="Set")
        # set_button.set_on_click_listener(self, callback_cb_name)

        item = gui.TableItem()
        item.add_child(0, param_name)
        row.add_child(0, item)

        # Dummy item
        item = gui.TableItem()
        item.add_child(1, "")
        row.add_child(1, item)

        item = gui.TableItem()
        item.add_child(2, text_input)
        row.add_child(2, item)
        text_input.style['display'] = 'block'
        text_input.style['margin'] = '10px auto'
        text_input.style['float'] = 'none'

        # Dummy item
        item = gui.TableItem()
        item.add_child(3, "")
        row.add_child(3, item)

        item = gui.TableItem()
        #item.add_child(4, set_button)
        item.add_child(4, "")
        row.add_child(4, item)

        return row, [text_input.set_text]

    def create_bool_row(self, param_name, description,
                        current_value, callback_cb):
        row = gui.TableRow()
        param_name = gui.Label(param_name,
                               width=NAME_L_SIZE,
                               height=FIELD_HEIGHT)
        param_name.attributes['title'] = description
        checkbox = gui.CheckBox(
            width=SLIDER_SIZE, height=FIELD_HEIGHT, checked=current_value)
        checkbox.set_on_change_listener(callback_cb)

        item = gui.TableItem()
        item.add_child(0, param_name)
        row.add_child(0, item)

        # Dummy item
        item = gui.TableItem()
        item.add_child(1, "")
        row.add_child(1, item)

        item = gui.TableItem()
        item.add_child(2, checkbox)
        row.add_child(2, item)
        # To center the checkbox
        checkbox.style['float'] = 'none'
        checkbox.style['margin'] = '0px auto'
        checkbox.style['display'] = 'block'

        # Dummy item
        item = gui.TableItem()
        item.add_child(3, "")
        row.add_child(3, item)

        # Dummy item
        item = gui.TableItem()
        item.add_child(4, "")
        row.add_child(4, item)

        return row, [checkbox.set_value]

    def create_enum_row(self, param_name, description, current_value,
                        callback_cb, enums):
        row = gui.TableRow()
        param_name = gui.Label(param_name,
                               width=NAME_L_SIZE,
                               height=FIELD_HEIGHT)
        param_name.attributes['title'] = description
        dropdown = gui.DropDown(width=SLIDER_SIZE, height=FIELD_HEIGHT)

        # Fill dropdown
        for idx, enum in enumerate(enums):
            description_enum = enum['description']
            value = enum['value']
            name = enum['name']
            type_enum = enum['type']
            ddi_text = name + " (" + str(value) + ")"
            ddi = gui.DropDownItem(
                ddi_text, width=EDIT1_SIZE, height=FIELD_HEIGHT)
            ddi.attributes['title'] = description_enum
            dropdown.add_child(value, ddi)
            if value == current_value:
                dropdown.select_by_key(value)

        item = gui.TableItem()
        item.add_child(0, param_name)
        row.add_child(0, item)

        # Dummy item
        item = gui.TableItem()
        item.add_child(1, "")
        row.add_child(1, item)

        item = gui.TableItem()
        item.add_child(2, dropdown)
        row.add_child(2, item)
        dropdown.style['display'] = 'block'
        dropdown.style['margin'] = '0px auto'
        dropdown.style['float'] = 'none'

        # Dummy item
        item = gui.TableItem()
        item.add_child(3, "")
        row.add_child(3, item)

        # Dummy item
        item = gui.TableItem()
        item.add_child(4, "")
        row.add_child(4, item)

        dropdown.set_on_change_listener(callback_cb)

        # return the row itself and the setter func list
        return row, [dropdown.select_by_key]


if __name__ == "__main__":
    rospy.init_node('web_dyn_reconf', anonymous=True)
    ips_list = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [
        [(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
    rospy.loginfo(
        "Web dynamic reconfigure initalizing on port 8090 and ips: " + str(ips_list))
    start(MyApp,
          # address="192.168.200.132",
          address="0.0.0.0",
          port=8090,
          multiple_instance=True,
          start_browser=False,
          debug=True)
