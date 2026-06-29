

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
        const se2_t   pose, 
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
        const se2_t   pose, 
        const PointCloud*   cloud)
{
    // static variables to keep track of scan count across calls
    static uint32_t      i;
    uint32_t k;

    PRINTF("pose_%lu = [%.2f,%.2f,%.3f]\n",
           i, pose.x, pose.y, pose.theta);
        
    PRINTF("scan_%lu = np.array([\n", i);
    
    for (k = 0; k < cloud->num_pts; k++) {

        PRINTF("\t\t[%.1f,%.1f],\n",
            cloud->points[k].x,
            cloud->points[k].y);
    }

    PRINTF("])\n");
    i++;
}


