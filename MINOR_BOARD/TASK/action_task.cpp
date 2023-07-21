#include "action_task.h"
#include "sucker.h"

extern DesSet DES_qzj;
int use_xiaban = 1;

void deal_with_message(void)
{
	if (use_xiaban)
	{
		memcpy(&DES_qzj.sucker_slide, efr6.num, 4);
		memcpy(&DES_qzj.sucker_lift, &efr6.num[4], 4);
	}
}

void action_detection(void)
{
	rate_monitor.temp_rate[4]++;
}
