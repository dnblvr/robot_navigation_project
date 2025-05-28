
#include "msp.h"
#include "inc/matrices.h"


#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "inc/coordinate_transform.h"
#include "inc/ransac.h"
#include "inc/graphslam.h"
#include "inc/rrt_star.h"






/**
 * main.c
 */
int main(void)
{
	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer




    // example_coord_transform();
    example_coord_transform_2();


    // example_ransac();
    // example_ransac_2();

	return 0;
}
