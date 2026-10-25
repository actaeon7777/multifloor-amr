"""Request EWMH fullscreen for an existing X11 window; no third-party modules."""
import ctypes
import ctypes.util
import sys


class ClientMessage(ctypes.Structure):
    _fields_ = [
        ('type', ctypes.c_int), ('serial', ctypes.c_ulong), ('send_event', ctypes.c_int),
        ('display', ctypes.c_void_p), ('window', ctypes.c_ulong),
        ('message_type', ctypes.c_ulong), ('format', ctypes.c_int),
        ('data', ctypes.c_long * 5),
    ]


class XEvent(ctypes.Union):
    _fields_ = [('xclient', ClientMessage), ('pad', ctypes.c_long * 24)]


def main():
    if len(sys.argv) != 2:
        raise SystemExit('Usage: fullscreen.py WINDOW_ID')
    window = int(sys.argv[1], 0)
    library = ctypes.util.find_library('X11')
    if not library:
        raise SystemExit('libX11 not found')
    x11 = ctypes.CDLL(library)
    display_type = ctypes.c_void_p
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = display_type
    x11.XDefaultRootWindow.argtypes = [display_type]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XInternAtom.argtypes = [display_type, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XSendEvent.argtypes = [display_type, ctypes.c_ulong, ctypes.c_int,
                              ctypes.c_long, ctypes.POINTER(XEvent)]
    x11.XSendEvent.restype = ctypes.c_int
    x11.XFlush.argtypes = [display_type]
    x11.XCloseDisplay.argtypes = [display_type]
    display = x11.XOpenDisplay(None)
    if not display:
        raise SystemExit('Cannot open the desktop X11 display')
    try:
        event = XEvent()
        event.xclient.type = 33  # ClientMessage
        event.xclient.display = display
        event.xclient.window = window
        event.xclient.message_type = x11.XInternAtom(display, b'_NET_WM_STATE', 0)
        event.xclient.format = 32
        event.xclient.data[0] = 1  # _NET_WM_STATE_ADD
        event.xclient.data[1] = x11.XInternAtom(display, b'_NET_WM_STATE_FULLSCREEN', 0)
        event.xclient.data[3] = 1  # source: application
        mask = (1 << 20) | (1 << 19)  # SubstructureRedirect | SubstructureNotify
        if not x11.XSendEvent(display, x11.XDefaultRootWindow(display), 0, mask,
                             ctypes.byref(event)):
            raise SystemExit('Fullscreen request failed')
        x11.XFlush(display)
    finally:
        x11.XCloseDisplay(display)


if __name__ == '__main__':
    main()
