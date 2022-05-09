class CaptureError(Exception):
    pass

class Motion(Exception):
    pass

class NoMotion(Exception):
    pass

class SendError(Exception):
    pass

class MqttError(Exception):
    pass

class EventReady(Exception):
    pass