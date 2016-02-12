
/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */

#include "timerCallback.h"
#include "sender.h"

unsigned int millisecondsElapsed = 0;

void vTimerCallback(TimerHandle_t pxTimer){
    millisecondsElapsed += 1; //Timer is called every 100ms
    app1SendTimerValToMsgQ(millisecondsElapsed);
	app1WriteMessage();

 }



/* *****************************************************************************
 End of File
 */
