#!/usr/bin/env python

import remi.gui as gui
from remi import start, App
import remi.server as server
import rospy
from dynamic_reconfigure import client as drc
from dynamic_reconfigure import find_reconfigure_services

DEFAULT_WIDTH = 700
DEFAULT_HEIGHT = 25



class MyApp(App):

    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def main(self, name='world'):
        # the arguments are	width - height - layoutOrientationHorizontal
        self.dyn_rec_client = None
        self.wid = gui.Widget(DEFAULT_WIDTH+100, 1000, False, 20)
        self.hor_servers = gui.Widget(500, 20, True, 10)
        self.bt = gui.Button(100, 30, 'Refresh list')
        self.bt.set_on_click_listener(self, 'refresh_servers')

        self.hor_servers.append(1, self.bt)
        self.refresh_servers()

        self.wid.append(1, self.hor_servers)

        # returning the root widget
        return self.wid

    def on_dropdown_change(self, value):
        print "Dropdown changed to: " + str(value)
        # TODO: Generate the dynamic reconfigure stuff here

        # If we had a previous client, disconnect it
        if self.dyn_rec_client is not None:
            self.dyn_rec_client.close()
        # Get new client
        # TODO: update all field when config_callback is called
        self.dyn_rec_client = drc.Client(value, config_callback=None)
        # Get a configuration which ensures we'll have the description too
        curr_conf = self.dyn_rec_client.get_configuration()
        params_list = self.dyn_rec_client.group_description['parameters']
        print "params list is: " + str(params_list)
        if params_list is not None:
            reconf_widget = gui.Widget(DEFAULT_WIDTH,
                                       DEFAULT_HEIGHT * len(params_list),
                                       False,
                                       10)
            for idx, param in enumerate(params_list):
                if param['type'] == 'int' and param['edit_method'] != '':
                    # Enum
                    pass
                elif param['type'] == 'int' or param['type'] == 'double':
                    param_name = param['name']
                    range_min = param['min']
                    range_max = param['max']
                    default_value = param['default']
                    # TODO: figure out what callback give it
                    if param['type'] == 'int':
                        step = 1
                    elif param['type'] == 'double':
                        step = 0.1
                    num_wid = self.create_num_widget(param_name, range_min,
                                                     range_max, default_value,
                                                     'callback_cb_name', step)
                    reconf_widget.append(idx, num_wid)
                elif param['type'] == 'str':
                    param_name = param['name']
                    default_value = param['default']
                    # TODO: figure out callback
                    str_wid = self.create_str_widget(param_name,
                                                     default_value,
                                                     'callback_cb_name')
                    reconf_widget.append(idx, str_wid)
                elif param['type'] == 'bool':
                    pass
        # TODO: hover text for fields
        self.wid.append(2, reconf_widget)

    def refresh_servers(self):
        self.dynamic_reconfigure_servers = find_reconfigure_services()
        rospy.loginfo("Found dynamic reconfigure servers:\n" +
                      str(self.dynamic_reconfigure_servers))
        self.dropdown = gui.DropDown(200, 20)
        choose_ddi = gui.DropDownItem(200, 20, "Choose server...")
        self.dropdown.append(0, choose_ddi)
        for idx, server_name in enumerate(self.dynamic_reconfigure_servers):
            ddi = gui.DropDownItem(200, 20, server_name)
            self.dropdown.append(idx+1, ddi)

        self.dropdown.set_on_change_listener(self, 'on_dropdown_change')
        # using ID 2 to update the dropdown
        self.hor_servers.append(2, self.dropdown)

    def create_num_widget(self, param_name, range_min, range_max,
                          default_value, callback_cb_name, step):
        horiz_container = gui.Widget(DEFAULT_WIDTH,
                                     DEFAULT_HEIGHT,
                                     True,  # Horizontal?
                                     20)
        param_name = gui.Label(100,
                               10,
                               param_name)
        min_val = gui.Label(1,
                            10,
                            str(range_min))
        range_slider = gui.Slider(300, 10,
                                  defaultValue=default_value, min=range_min,
                                  max=range_max,
                                  step=step)
        range_slider.set_on_change_listener(self, callback_cb_name)
        max_val = gui.Label(1,
                            10,
                            str(range_max))
        spin_val = gui.SpinBox(80, DEFAULT_HEIGHT,
                               defaultValue=default_value,
                               min=range_min, max=range_max, step=step)
        spin_val.set_on_change_listener(self, callback_cb_name)
        horiz_container.append(0, param_name)
        horiz_container.append(1, min_val)
        horiz_container.append(2, range_slider)
        horiz_container.append(3, max_val)
        horiz_container.append(4, spin_val)

        return horiz_container

    def create_str_widget(self, param_name, default_value, callback_cb_name):
        horiz_container = gui.Widget(DEFAULT_WIDTH,
                                     DEFAULT_HEIGHT,
                                     True,
                                     10)
        param_name = gui.Label(100,
                               10,
                               param_name)
        text_input = gui.TextInput(200, DEFAULT_HEIGHT)
        text_input.set_text(default_value)
        text_input.set_on_change_listener(self, callback_cb_name)

        horiz_container.append(0, param_name)
        horiz_container.append(1, text_input)

        return horiz_container

    def create_bool_widget(self):
        # TODO: Add a checkbox class to the REMI project using the two images
        # meanwhile we can use two images
        pass

    def create_enum_widget(self):
        pass


if __name__ == "__main__":
    rospy.init_node('web_dyn_reconf')
    app = server.Server(MyApp, start=True, port=8081,
                        multiple_instance=True)
    rospy.spin()
