#!/usr/bin/env python

import remi.gui as gui
import remi.server as server
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

    def main(self, name='world'):
        # the arguments are	width - height - layoutOrientationHorizontal
        self.dyn_rec_client = None
        #self.wid = gui.Widget(DEFAULT_WIDTH+100, 1000, False, 20)
        self.wid = gui.Widget(-1, -1, gui.Widget.LAYOUT_VERTICAL, 20)
        #self.hor_servers = gui.Widget(500, 20, True, 10)

        #self.hor_servers = gui.Widget(-1, -1, gui.Widget.LAYOUT_HORIZONTAL, 20)
        self.hor_servers = gui.Widget(-1, -1, gui.Widget.LAYOUT_HORIZONTAL, 20)
        #self.bt = gui.Button(100, 30, 'Refresh list')
        self.bt = gui.Button(-1, -1, 'Refresh Dynamic Reconfigure Servers list')
        self.bt.set_on_click_listener(self, 'refresh_servers')

        self.hor_servers.append(1, self.bt)
        # This makes the button not be left
        self.bt.style['display'] = 'block'
        self.bt.style['margin'] = '10px auto'
        self.bt.style['float'] = 'none'

        self.refresh_servers()
        #self.hor_servers.style['padding-bottom'] = '20px'

        # returning the root widget
        return self.wid

    def dyn_rec_conf_callback(self, new_config):
        rospy.loginfo("Updating GUI with:\n" + str(new_config))
        for k in new_config:
            if self.gui_set_funcs_for_param.has_key(k):
                for set_func in self.gui_set_funcs_for_param[k]:
                    set_func(new_config[k])

    def on_dropdown_change(self, value):
        print "Dropdown changed to: " + str(value)

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
            #table = gui.Table(DEFAULT_WIDTH, DEFAULT_HEIGHT * len(params_list))
            table = gui.Table(-1, -1)
            row = gui.TableRow()
            item = gui.TableTitle()
            item.append(str(id(item)), 'Param name')
            row.append(str(id(item)), item)
            item = gui.TableTitle()
            item.append(str(id(item)), 'Min')
            row.append(str(id(item)), item)
            item = gui.TableTitle()
            item.append(str(id(item)), 'Edit')
            row.append(str(id(item)), item)
            item = gui.TableTitle()
            item.append(str(id(item)), 'Max')
            row.append(str(id(item)), item)
            item = gui.TableTitle()
            item.append(str(id(item)), 'Edit2')
            row.append(str(id(item)), item)
            table.append(str(id(row)), row)

            for idx, param in enumerate(params_list):
                if param['edit_method'] != '':
                    # Enum
                    param_name = param['name']
                    # WTF, really? the full enum dict is actually a string?
                    enum_dict_as_str = param['edit_method']
                    enum_dict = literal_eval(enum_dict_as_str)
                    description = enum_dict['enum_description']
                    current_value = curr_conf[param_name]
                    print "CURRENT VALUE OF ENUM IS: " + str(current_value)
                    enums = enum_dict['enum']
                    enum_type = enums[0]['type'] # there must be at least one enum
                    # Create dynamically a method to be called
                    method_name = self.add_cb_to_class_by_param_name_and_type(
                                                                    param_name,
                                                                    'enum',
                                                                    enum_type)
                    enum_wid, set_funcs = self.create_enum_row(param_name,
                                                               description,
                                                               current_value,
                                                               method_name,
                                                               enums)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.append(idx, enum_wid)
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
                    method_name = self.add_cb_to_class_by_param_name_and_type(
                                                                    param_name,
                                                                    'digit')

                    num_wid, set_funcs = self.create_num_row(param_name,
                                                             description,
                                                             range_min,
                                                             range_max,
                                                             current_value,
                                                             method_name, step)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.append(idx, num_wid)
                elif param['type'] == 'str':
                    param_name = param['name']
                    current_value = curr_conf[param_name]
                    description = param['description']
                    # Create dynamically a method to be called
                    method_name = self.add_cb_to_class_by_param_name_and_type(
                                                                    param_name,
                                                                    'string')
                    str_wid, set_funcs = self.create_str_row(param_name,
                                                             description,
                                                             current_value,
                                                             method_name)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.append(idx, str_wid)
                elif param['type'] == 'bool':
                    param_name = param['name']
                    current_value = curr_conf[param_name]
                    description = param['description']
                    # Create dynamically a method to be called
                    method_name = self.add_cb_to_class_by_param_name_and_type(
                                                                    param_name,
                                                                    'bool')
                    bool_wid, set_funcs = self.create_bool_row(param_name,
                                                               description,
                                                               current_value,
                                                               method_name)
                    self.gui_set_funcs_for_param[param_name] = set_funcs
                    table.append(idx, bool_wid)


        self.wid.append(2, table)
        # This must be done later on! HACK! (append sets a margin)
        # This makes the table not stick left, but float in the middle
        table.style['margin'] = '10px auto'

        # Once the GUI is setup, setup the callback for new configs
        self.dyn_rec_client.set_config_callback(self.dyn_rec_conf_callback)

    def add_cb_to_class_by_param_name_and_type(self, param_name, param_type, enum_type=None):
        # Create dynamically a method to be called
        def make_method(parameter_name):
            def _method(new_value):
                print "Cb for param '" + parameter_name + "' called."
                print "We got value: " + str(new_value)
                if param_type == 'digit' or param_type == 'string':
                    value_to_set = new_value
                elif param_type == 'enum':
                    # Fix the value as per type
                    value = new_value[new_value.find("(")+1:new_value.find(")")]
                    if enum_type == 'int':
                        value_to_set = int(value)
                    elif enum_type == 'str':
                        value_to_set = value
                    elif enum_type == 'float':
                        value_to_set = float(value)
                elif param_type == 'bool':
                    # Fix the value as its returned as a string and we need a boolean
                    if new_value == 'true':
                        value_to_set = True
                    elif new_value == 'false':
                        value_to_set = False
                    else:
                        rospy.logerr("Error parsing bool parameter in dynamic callback of " + parameter_name)
                self.dyn_rec_client.update_configuration({parameter_name: value_to_set})

            return _method

        method_name = "callback_func_for_" + param_name
        cb_method = make_method(param_name)
        setattr(self, method_name, cb_method)
        return method_name

    def refresh_servers(self):
        self.dynamic_reconfigure_servers = find_reconfigure_services()
        rospy.loginfo("Found dynamic reconfigure servers:\n" +
                      str(self.dynamic_reconfigure_servers))
        #self.dropdown = gui.DropDown(200, 20)
        self.dropdown = gui.DropDown(-1, -1)
        #choose_ddi = gui.DropDownItem(200, 20, "Choose server...")
        choose_ddi = gui.DropDownItem(-1, -1, "Choose server...")
        self.dropdown.append(0, choose_ddi)
        for idx, server_name in enumerate(self.dynamic_reconfigure_servers):
            #ddi = gui.DropDownItem(200, 20, server_name)
            ddi = gui.DropDownItem(-1, -1, server_name)
            self.dropdown.append(idx+1, ddi)

        self.dropdown.set_on_change_listener(self, 'on_dropdown_change')
        # using ID 2 to update the dropdown
        self.hor_servers.append(2, self.dropdown)
        # This makes the dropdown not be left
        self.dropdown.style['display'] = 'block'
        self.dropdown.style['margin'] = '10px auto'
        self.dropdown.style['float'] = 'none'
        self.wid.append(1, self.hor_servers)


    def create_num_row(self, param_name, description, range_min, range_max,
                       current_value, callback_cb_name, step):
        row = gui.TableRow()
        param_name = gui.Label(NAME_L_SIZE,
                               FIELD_HEIGHT,
                               param_name)
        param_name.attributes['title'] = description
        min_val = gui.Label(MIN_L_SIZE,
                            FIELD_HEIGHT,
                            str(range_min))
        range_slider = gui.Slider(SLIDER_SIZE, FIELD_HEIGHT,
                                  defaultValue=current_value, min=range_min,
                                  max=range_max,
                                  step=step)
        range_slider.set_on_change_listener(self, callback_cb_name)
        max_val = gui.Label(MAX_L_SIZE,
                            FIELD_HEIGHT,
                            str(range_max))
        spin_val = gui.SpinBox(EDIT2_SIZE, FIELD_HEIGHT,
                               defaultValue=current_value,
                               min=range_min, max=range_max, step=step)
        # https://github.com/dddomodossola/remi/issues/49
        # Added 46 as it's dot so we allow floating point values
        spin_val.attributes[spin_val.EVENT_ONKEYPRESS] = 'return event.charCode >= 48 && event.charCode <= 57 || event.charCode == 46 || event.charCode == 13'
        spin_val.set_on_change_listener(self, callback_cb_name)

        item = gui.TableItem()
        item.append(0, param_name)
        row.append(0, item)

        item = gui.TableItem()
        item.append(1, min_val)
        row.append(1, item)
        min_val.style['float'] = 'none'
        min_val.style['text-align'] = 'center'

        item = gui.TableItem()
        item.append(2, range_slider)
        row.append(2, item)

        item = gui.TableItem()
        item.append(3, max_val)
        row.append(3, item)
        max_val.style['float'] = 'none'
        max_val.style['text-align'] = 'center'

        item = gui.TableItem()
        item.append(4, spin_val)
        row.append(4, item)
        spin_val.style['display'] = 'block'
        spin_val.style['margin'] = '10px auto'
        spin_val.style['float'] = 'none'

        return row, [range_slider.set_value, spin_val.set_value]

    def test_cb(self, *args):
        rospy.loginfo("We got a cb with args: " + str(args))
        rospy.loginfo("Which the arg is of type: ")
        if len(args) >= 1:
            rospy.loginfo(str(type(args[0])))
        #self.dyn_rec_client.update_configuration()

    def create_str_row(self, param_name, description,
                       current_value, callback_cb_name):
        row = gui.TableRow()
        param_name = gui.Label(NAME_L_SIZE,
                               FIELD_HEIGHT,
                               param_name)
        param_name.attributes['title'] = description
        text_input = gui.TextInput(SLIDER_SIZE, FIELD_HEIGHT, single_line=True)
        text_input.set_text(current_value)

        text_input.set_on_enter_listener(self, callback_cb_name)

        # set_button = gui.Button(80, DEFAULT_HEIGHT, text="Set")
        # set_button.set_on_click_listener(self, callback_cb_name)

        item = gui.TableItem()
        item.append(0, param_name)
        row.append(0, item)

        # Dummy item
        item = gui.TableItem()
        item.append(1, "")
        row.append(1, item)

        item = gui.TableItem()
        item.append(2, text_input)
        row.append(2, item)
        text_input.style['display'] = 'block'
        text_input.style['margin'] = '10px auto'
        text_input.style['float'] = 'none'

        # Dummy item
        item = gui.TableItem()
        item.append(3, "")
        row.append(3, item)

        item = gui.TableItem()
        #item.append(4, set_button)
        item.append(4, "")
        row.append(4, item)

        return row, [text_input.set_text]

    def create_bool_row(self, param_name, description,
                        current_value, callback_cb_name):
        row = gui.TableRow()
        param_name = gui.Label(NAME_L_SIZE,
                               FIELD_HEIGHT,
                               param_name)
        param_name.attributes['title'] = description
        checkbox = gui.CheckBox(SLIDER_SIZE, FIELD_HEIGHT, checked=current_value)
        checkbox.set_on_change_listener(self, callback_cb_name)

        item = gui.TableItem()
        item.append(0, param_name)
        row.append(0, item)

        # Dummy item
        item = gui.TableItem()
        item.append(1, "")
        row.append(1, item)

        item = gui.TableItem()
        item.append(2, checkbox)
        row.append(2, item)
        # To center the checkbox
        checkbox.style['float'] = 'none'
        checkbox.style['margin'] = '0px auto'
        checkbox.style['display'] = 'block'

        # Dummy item
        item = gui.TableItem()
        item.append(3, "")
        row.append(3, item)

        # Dummy item
        item = gui.TableItem()
        item.append(4, "")
        row.append(4, item)

        return row, [checkbox.set_value]

    def create_enum_row(self, param_name, description, current_value,
                        callback_cb_name, enums):
        row = gui.TableRow()
        param_name = gui.Label(NAME_L_SIZE,
                               FIELD_HEIGHT,
                               param_name)
        param_name.attributes['title'] = description
        dropdown = gui.DropDown(SLIDER_SIZE, FIELD_HEIGHT)

        # Fill dropdown
        for idx, enum in enumerate(enums):
            description_enum = enum['description']
            value = enum['value']
            name = enum['name']
            type_enum = enum['type']
            ddi_text = name + " (" + str(value) + ")"
            ddi = gui.DropDownItem(EDIT1_SIZE, FIELD_HEIGHT,
                                   ddi_text)
            ddi.attributes['title'] = description_enum
            dropdown.append(value, ddi)
            if value == current_value:
                dropdown.select_by_key(value)

        item = gui.TableItem()
        item.append(0, param_name)
        row.append(0, item)

        # Dummy item
        item = gui.TableItem()
        item.append(1, "")
        row.append(1, item)

        item = gui.TableItem()
        item.append(2, dropdown)
        row.append(2, item)
        dropdown.style['display'] = 'block'
        dropdown.style['margin'] = '0px auto'
        dropdown.style['float'] = 'none'

        # Dummy item
        item = gui.TableItem()
        item.append(3, "")
        row.append(3, item)

        # Dummy item
        item = gui.TableItem()
        item.append(4, "")
        row.append(4, item)

        dropdown.set_on_change_listener(self, callback_cb_name)

        # return the row itself and the setter func list
        return row, [dropdown.select_by_key]


if __name__ == "__main__":
    rospy.init_node('web_dyn_reconf')
    server.DEBUG_MODE = 2
    app = server.Server(MyApp, start=True,
                        # address="192.168.200.132",
                        address="0.0.0.0",
                        port=8081,
                        multiple_instance=True,
                        start_browser=False)
    rospy.spin()
