// This is for MFC wnd object only. 
// if wanted to extends to any windows, need to change a little
// 
#pragma once

#include "afxwin.h"

#include <boost/shared_ptr.hpp>

#include "serial_com_messenger_hook.h"

namespace SerialCom {

// window that that has a message loop and is cwnd type (TODO!!! verify cwnd type)
// MsgHandler: The class that provides Event handler.
template<class MsgHandler> 
class WinMessenger : public serial_com::MessengerHook<MsgHandler> {
    friend typename MsgHandler;
public:
    static const int WM_SERIAL_COM_IO = WM_APP + 108; 
    // MsgHandler: the object that provides event handler;
    // win: the window that provides event loops. This can be null If handler is also the event loop
    WinMessenger(MsgHandler* handler, CWnd* win = NULL) 
        : MessengerHook<MsgHandler>(handler) {
            if (win == NULL) {
                this->win = reinterpret_cast<CWnd*>(handler);       // any safe way?
            }
            else {
                this->win = win;
            }
    }
    ~WinMessenger() {
    }

    LRESULT OnSerialComIO(WPARAM wParam, LPARAM lparam);           // main thread communication event pump
    virtual void Poster(serial_com::BaseCompletionCarrier* instance, serial_com::DoCompletePointer doComplete) {
        if (win == NULL) {
            delete instance;        // could it be wrapped with shared pointer?
            return;
        }
        // message tareget must be a MFC window
        HWND hWnd = win->GetSafeHwnd();
        if (!hWnd) {
            delete instance;
            return;
        }
        ::PostMessage(hWnd, WM_SERIAL_COM_IO, (WPARAM)instance, (LPARAM)doComplete);
    }

private:
    CWnd* win;
};

template<class MsgWin> 
inline LRESULT WinMessenger<MsgWin>::OnSerialComIO(WPARAM wParam, LPARAM lParam) {
   serial_com::BaseCompletionCarrier* base = reinterpret_cast<serial_com::BaseCompletionCarrier*>(wParam);
   assert(lParam);
   serial_com::DoCompletePointer func = (serial_com::DoCompletePointer)(lParam);
   (*func)(base);
   return (LRESULT)0;
}

}