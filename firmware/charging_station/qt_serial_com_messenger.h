#ifndef __QT_SERIAL_COM_MESSENGER_H__
#define __QT_SERIAL_COM_MESSENGER_H__

#include <boost/shared_ptr.hpp>

#include <QCoreApplication>

#include "serial_com_messenger_hook.h"

#include <QCoreApplication>

namespace SerialCom {

class CommQEvent : public QEvent {
public:
    CommQEvent(serial_com::BaseCompletionCarrier* instance, serial_com::DoCompletePointer completeFunc, size_t eventType);
    virtual  ~CommQEvent ();
    serial_com::BaseCompletionCarrier* instance;
    serial_com::DoCompletePointer func;
    // Not sure copyable or not!!!
};

/// <summary> Constructor. </summary>
/// <param name="evt">       The underlying event. </param>
/// <param name="eventType"> Qt event type </param>
inline CommQEvent::CommQEvent(serial_com::BaseCompletionCarrier* instance, serial_com::DoCompletePointer func, size_t eventType)
    : instance(instance), func(func), QEvent(QEvent::Type(eventType)) {
}

/// <summary> Default constructor. </summary>
inline CommQEvent::~CommQEvent () {
}

template<typename Handler>
void PostWindow(QObject *receiver, size_t eventType, const Handler& hdl) {
    typedef CompletionCarrier<Handler> CompletionCarrierInstance;
    CompletionCarrierInstance* instance = new CompletionCarrierInstance(hdl);
    DoCompletePointer func = CompletionCarrierInstance::DoComplete;
    QCoreApplication::postEvent(receiver, new CommQEvent(instance, func, QEvent::Type(eventType)));
}

template<class EventFilter>     // Qtobject that can that has event filter ability
class QtMessenger : public serial_com::MessengerHook<EventFilter> {
    friend typename EventFilter;
public:
    QtMessenger(EventFilter* receiver)
        : MessengerHook<EventFilter>(receiver)
        , receiver(receiver) {
        assert(receiver);
    }

    ~QtMessenger() {
    }

    bool QtMessenger::eventFilter(QObject *obj, QEvent *event) {           // main thread communication event pump
        if (event->type() != EVENT_COMM_IO) {
            return false;
        }

        CommQEvent* cqEvent = dynamic_cast<CommQEvent*>(event);
        assert(cqEvent);

       assert(cqEvent->func);
       assert(cqEvent->instance);

       (*cqEvent->func)(cqEvent->instance);

        return true;
    }

    virtual void Poster(serial_com::BaseCompletionCarrier* instance, serial_com::DoCompletePointer doComplete) {
        QCoreApplication::postEvent(receiver, new CommQEvent(instance, doComplete, QEvent::Type(EVENT_COMM_IO)));
    }

    static const size_t EVENT_COMM_IO = QEvent::User + 103;
private:
    EventFilter* receiver;

};

}

#endif   // __QT_SERIAL_COM_MESSENGER_H__
