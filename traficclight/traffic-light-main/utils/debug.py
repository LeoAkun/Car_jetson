class Debug:
    def __init__(self, debugcfg):
        self.islog = debugcfg['islog']
    
    def log(self, *args):
        if self.islog:
            print(*args)