#include <val_task3/val_task3.h>

FSM(val_task3){

    FSM_STATES{

        STATE_INIT,
        STATE_END,
        STATE_ERROR

    }

    FSM_START(STATE_INIT);

    FSM_BGN{
        FSM_STATE(STATE_INIT){
            FSM_CALL_TASK(STATE_INIT)

                    FSM_TRANSITIONS{

                FSM_ON_EVENT("/INIT_RETRY", FSM_NEXT(STATE_INIT))
                        FSM_ON_EVENT("/INIT_SUCESSFUL", FSM_NEXT(STATE_END))
                        FSM_ON_EVENT("/INIT_FAILED", FSM_NEXT(STATE_ERROR))
            }
        }
        FSM_STATE(STATE_END) {

            FSM_CALL_TASK(STATE_END)

                    FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/END_SUCESSFUL", FSM_NEXT(STATE_END))
            }
        }

        FSM_STATE(STATE_ERROR){

            FSM_CALL_TASK(STATE_ERROR)

                    FSM_TRANSITIONS
            {
                FSM_ON_EVENT("/RESTART", FSM_NEXT(STATE_INIT))
            }
        }

    }
    FSM_END
}
