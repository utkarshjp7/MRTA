class LogLevel:
    OFF=-1, 
    ERROR=0,
    WARN=1,
    INFO=2,
    DEBUG=3

class Logger:

    def __init__(self, level):        
        self.level = level

    def debug(self, log):
        if self.level >= LogLevel.DEBUG:
            print log

    def info(self, log):
        if self.level >= LogLevel.INFO:
            print log

    def warn(self, log):
        if self.level >= LogLevel.WARN:
            print log

    def error(self, log):
        if self.level >= LogLevel.ERROR:
            print error

    def special(self, log):
        print log