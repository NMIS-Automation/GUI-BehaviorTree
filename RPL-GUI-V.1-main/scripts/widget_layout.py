from json_class import *


class widget_handler:
    def __init__(self,filename = None, view_widgets = []):
        self.success = False
        self.topic_layout = {}
        if len(view_widgets)==0:
            return
        for view in view_widgets:
            self.topic_layout[view] = None      
        self.json_ = json_()
        self.load_topics(filename=filename)
                    
    def add_topic(self,topic_name, element_name, topic_type,label, n_var = None):
        self.topic_layout[element_name] = {'name': topic_name, 'type': topic_type, 'label': label, 'n_var': n_var}
        
    def load_topics(self, filename = None):
        self.success = False
        if filename is not None:
            self.config = self.json_.load(filename)
            if self.config is not None:
                self.success = True

    def save_topics(self, filename = None):
        if filename is None:
            filename = 'widget_config'
        filename+='.json'
        self.json_.save(filename, self.topic_layout)
    




