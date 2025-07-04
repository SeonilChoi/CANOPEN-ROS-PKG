from canopen_rqt.canopen_widget import CANopenWidget
from rqt_gui_py.plugin import Plugin

class CANopenController(Plugin):
    def __init__(self, context):
        super().__init__(context)
        
        self.setObjectName('CANopenController')
        self.widget = CANopenWidget(context.node)
        serial_number = context.serial_number()
        if serial_number >= 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + ' ({0})'.format(serial_number))
        context.add_widget(self.widget)

    def shutdown_plugin(self):
        print('Shutdown the RQt.')
        self.widget.shutdown_widget()