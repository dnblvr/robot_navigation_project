

#include <data_structures.h>

#ifdef PROCESSING_EXPECTED_OUTPUTS

void processing4_print(
        const se2_t   pose, 
        const PointCloud*   cloud)
{
    uint32_t k;

    PRINTF("POSE,%.2f,%.2f,%.3f\n",
           pose.x, pose.y, pose.theta);

    PRINTF("SCAN_START\n");

    for (k = 0; k < cloud->num_pts; k++)
    {
        PRINTF("P,%.2f,%.2f\n",
               cloud->points[k].x,
               cloud->points[k].y);
    }

    PRINTF("SCAN_END\n");
}

#endif


void C_format_print(
        const se2_t         pose, 
        const PointCloud*   cloud)
{
    // static variables to keep track of scan count across calls
    static uint32_t      i;
    uint32_t k;

    PRINTF("se2_t pose_%lu = {%.2f,%.2f,%.3f};\n",
           i, pose.x, pose.y, pose.theta);
        
    PRINTF("PointCloud scan_%lu = {\n\t.points = {\n", i);
    
    for (k = 0; k < cloud->num_pts; k++) {

        PRINTF("\t\t{%.2ff,%.2ff},\n",
            cloud->points[k].x,
            cloud->points[k].y);
    }

    PRINTF("},\n.num_pts = %lu};\n", cloud->num_pts);
    i++;
}


void numpy_format_print(
        const se2_t         pose, 
        const PointCloud*   cloud)
{
    uint32_t k;

    static uint32_t counter = 0;

    PRINTF("pose_%lu = [%.2f, %.2f, %.3f]\n", counter,
           pose.x, pose.y, pose.theta);
        
    PRINTF("scan_%lu = np.array([\n", counter);
    
    for (k = 0; k < cloud->num_pts; k++) {

        PRINTF("\t[%.2f, %.2f],\n",
            cloud->points[k].x,
            cloud->points[k].y);
    }

    PRINTF("])\n");

    counter++;
}




// ────────────────────────────────────────────────────────────────────────────
//
//  TIMING HELPERS
//
// ────────────────────────────────────────────────────────────────────────────

#ifdef __IMXRT1062__

void start_free_running_timer()
{
    ARM_DEMCR      |=  ARM_DEMCR_TRCENA;
    ARM_DWT_CYCCNT  =  0;
    ARM_DWT_CTRL   |=  ARM_DWT_CTRL_CYCCNTENA;
}

uint32_t get_elapsed_time_us()
{
    uint32_t elapsed_cycles = ARM_DWT_CYCCNT;
    return (uint32_t)((uint64_t)elapsed_cycles * 1000000ULL / 600000000ULL);
}

#endif  // __IMXRT1062__