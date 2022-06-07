/*
 * File:         statuc_sensor_net_controller.c
 * Description:  This is a supervisor node to place some static sensors and log their values
 * Author:       Faezeh Rahbar - Jan 2020
 * Note:         
 *               
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>

#include "SampleBuffer.h"
#include "tinyxml2.h"

#define PI 3.14159265359
#define TIME_STEP           64 //adjusts the speed (ms) // Default value 64
#define WAITING_TIME        60 // in seconds
#define SENSOR_NUM_MAX      250
#define NB_SENSORS_X        16
#define NB_SENSORS_Y        5
#define NB_SENSORS_Z        3
#define ODOR_FILTER_LENGTH  150 // Default value 150 -> ~10sec //number of measurements before averaging
#define ITERATIONS          1
#define RANDOM0_GRID1       1
#define RANDOM_ITERATIONS   0
#define RECORDING_TIME      5  // seconds
#define CHANGE_NB_SENSORS   false

#define XMIN 2.75
#define XMAX 12.7418
#define XSTEP 0.1586
#define XNUM 64

#define YMIN 0.01
#define YMAX 3.9916
#define YSTEP 0.0632
#define YNUM 64

using namespace tinyxml2;

WbFieldRef root_children_field;
WbDeviceTag odor_tag[SENSOR_NUM_MAX];
WbDeviceTag wind_tag[SENSOR_NUM_MAX];
SampleBuffer* sampleBuffer = new SampleBuffer[SENSOR_NUM_MAX];
int sensor_number = 0;
double sensor_position[SENSOR_NUM_MAX][3] = {{0}};
double source_position[3] = {15, 2, 1};
int total_nbr_iteration = 0;
int iteration = 0;

void change_number_of_sensors();
void set_sensor_position(int i, double * position);
void read_sensor_positions_from_file();
void place_sensor(int sensor_ID, double* position);
void wind_read(int i, double* intensity, double* angle);

void init(){
    int i = 0;

    wb_robot_init();
    srand((long)time(NULL));

    // find total number of itrations
    total_nbr_iteration = RECORDING_TIME*1000/TIME_STEP;
    printf("total_nbr_iteration : %d\n", total_nbr_iteration);
      sensor_number = NB_SENSORS_X*NB_SENSORS_Y*NB_SENSORS_Z;

    //Get static sensor network
    WbNodeRef root_node = wb_supervisor_node_get_self();
  
    //Define position of new node inside the children field
    root_children_field = wb_supervisor_node_get_field(root_node, "children");
    
    // Change number of sensors
    if(CHANGE_NB_SENSORS)
        change_number_of_sensors();
    
    //read_sensor_positions_from_file();
    //create new sensor nodes
    for (i=0; i<SENSOR_NUM_MAX; i++){
        //find the initial sensor position
        set_sensor_position(i, sensor_position[i]);
        //apply the position to both the odor sensor and wind sensor of ID i
        place_sensor(i, sensor_position[i]);
        //activate the sensors
        //odor
        char tag_name[256];
        sprintf(tag_name,"sensor_odor_%d",i);
        odor_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(odor_tag[i], TIME_STEP);
        //wind
        sprintf(tag_name,"sensor_wind_%d",i);
        wind_tag[i] = wb_robot_get_device(tag_name);
        wb_receiver_enable(wind_tag[i],TIME_STEP);
        //set buffer length for all sensors
        sampleBuffer[i].set_buffer_length(total_nbr_iteration);
    }

    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_node_get_from_def("SOURCE_ODOR_0"),"translation"), source_position);
}


void set_sensor_position(int i, double * position){
      // Static 2D grid (3x15 = 45 sensors)
      if(i < sensor_number){
          position[0] = (double)((i/NB_SENSORS_Y)%NB_SENSORS_X)/2 + 7.5;
          position[1] = (double)(i%NB_SENSORS_Y)/2 + 1;
          position[2] = (double)(i/(NB_SENSORS_X*NB_SENSORS_Y))/5 + 1.0;
      }
      else{
          position[0] = 0;
          position[1] = 0;
          position[2] = 0.1;
      }
      
      //printf("Sensor %d position is %f %f %f\n", i, position[0], position[1], position[2]);

    /*if(total_nbr_iteration - iteration <= RANDOM_ITERATIONS){
        //random
        position[0] = ( (rand()/(double)RAND_MAX) * (XMAX-XMIN) ) + XMIN;
        position[1] = ( (rand()/(double)RAND_MAX) * (YMAX-YMIN) ) + YMIN;
        position[2] =  0.1;
    }else{
        //grid        
        int index = i + iteration * SENSOR_NUM_MAX;
        if (index >= XNUM*YNUM){
            if(sensor_number == SENSOR_NUM_MAX)
                sensor_number = i;
            position[0] = 0;
            position[1] = 0;
            position[2] = 0.1;
        }
        else{
            position[0] = XMIN + floor(index/YNUM) * XSTEP;
            position[1] = YMIN + (index%YNUM) * YSTEP;
            position[2] = 0.1;
        }
        //printf("i %d index %d position %f %f \n", i, index, position[0], position[1]);
    }*/
}

void read_sensor_positions_from_file(){
    int res = 0;
    FILE *pFile = fopen("../../data/sensor_position.csv", "r");
    //read the number of sensors
    res = fscanf(pFile, "%d", &sensor_number);
    if(res == 0)
        printf("/!\\ no sensor number found!!! \n");
    //read the positions
    for(int i=0; i<sensor_number; i++){
        res = fscanf(pFile, "%lf, %lf", &(sensor_position[i][0]), &(sensor_position[i][1]));
        sensor_position[i][2] = 0.1;
    }
    //set other sensors position to 0
    for(int i=sensor_number; i<SENSOR_NUM_MAX; i++){
        sensor_position[i][0] = 0.0;
        sensor_position[i][1] = 0.0;
        sensor_position[i][2] = 0.0;
    }
}

void change_number_of_sensors(){
    //open txt file and make the necessary modifications, then copy paste in the world file
    FILE * wbo_file = fopen("../../protos/sensor_odor_receiver.wbo", "w");
    fprintf(wbo_file, "#VRML_OBJ R2021b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file, "DEF SENSOR_ODOR_%d Receiver {\n\
        translation %f %f 0.1\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                        diffuseColor 0 1 0\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_ODOR Cylinder {\n\
                    height 0.02\n\
                    radius 0.01\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_odor_%d\"\n\
        boundingObject USE CYLINDER_ODOR\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i);
    }
    fclose(wbo_file);
    /*
    //open txt file and make the necessary modifications
    FILE * wbo_file_w = fopen("../../protos/sensor_wind_receiver.wbo", "w");
    fprintf(wbo_file_w, "#VRML_OBJ R2019b utf8\n");
    for (int i = 0; i < SENSOR_NUM_MAX; i++){
        fprintf(wbo_file_w, "DEF SENSOR_WIND_%d Receiver {\n\
        translation %f %f 0.2\n\
        rotation 1 0 0 -1.5708\n\
        children [\n\
            Shape {\n\
                appearance Appearance {\n\
                    material Material {\n\
                        diffuseColor 1 0 0\n\
                        emissiveColor 1 0 0\n\
                    }\n\
                }\n\
                geometry DEF CYLINDER_WIND Cylinder {\n\
                    height 0.02\n\
                    radius 0.02\n\
                }\n\
            }\n\
        ]\n\
        name \"sensor_wind_%d\"\n\
        boundingObject USE CYLINDER_WIND\n\
        physics Physics {\n\
        }\n\
        channel %d\n\
        }\n", i, 3.0+(rand()/(double)RAND_MAX)*10.0, (rand()/(double)RAND_MAX)*4.0, i, i+SENSOR_NUM_MAX);
    }
    fclose(wbo_file_w);
    */

    //add the node to the scene tree
    wb_supervisor_field_import_mf_node(root_children_field, -1, "../../protos/sensor_odor_receiver.wbo");
    
    //exit to start again
    printf("number of sensors changed successfully, now close and run again the simulation.\n");
    exit(0);
}

double odor_read(int i){
    double odor_sample = 0;
    while (wb_receiver_get_queue_length(odor_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(odor_tag[i]) == sizeof(double));
        odor_sample = *(const double *)wb_receiver_get_data(odor_tag[i]);
        wb_receiver_next_packet(odor_tag[i]);
    }
    //printf("odor measurement : %f \n",odor_measurement);
    
    
    return odor_sample;
}

void wind_read(int i, double* intensity, double* angle){
    struct vect3D{
        double x;
        double y;
        double z;
    };
    struct vect3D v = {0};
    while (wb_receiver_get_queue_length(wind_tag[i]) > 0) {
        assert(wb_receiver_get_data_size(wind_tag[i]) == sizeof(struct vect3D));
        v = *(const struct vect3D*)wb_receiver_get_data(wind_tag[i]);
        wb_receiver_next_packet(wind_tag[i]);
    }
    //convert the 3D vector from the sensor to intensity and heading
    *intensity = sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    *angle = atan2(v.y, v.x)/PI*180; //degree
    //keep the 3D vector
    //*ux = v.x; 
    //*uy = v.y;
    //*uz = v.z;

}

void place_sensor(int sensor_ID, double* position){
    //wind sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),"translation"), position);
    //odor sensor
    wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID+SENSOR_NUM_MAX),"translation"), position);
    //correct orientation for wind sensors /!\ determines the origin for wind angle! should match with Khepera IV
    const double rotation_wind_sensors[] = {0, 0.70710528118436, -0.707108281185553, -3.1415923071795864};
    wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(wb_supervisor_field_get_mf_node(root_children_field, sensor_ID),"rotation"), rotation_wind_sensors);
}

bool still_wait(){
    //place one sensor at the end of the tunnel to measure the waiting time
    double tmp_pos[3] = {0.1,0,0.1};
    wind_read(SENSOR_NUM_MAX-1, &sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_intensity, &sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle);
    //printf("wind angle %f, tan %f \n", sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle, tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle));
    tmp_pos[1] = source_position[1] - tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle) * (source_position[0] - tmp_pos[0]);
    //printf("y %f\n", tmp_pos[1]);
    if(tmp_pos[1] > YMAX+0.5)
        tmp_pos[1] = YMAX+0.5;
    if(tmp_pos[1] < YMIN-0.5)
        tmp_pos[1] = YMIN-0.5;
    //printf("y %f\n", tmp_pos[1]);
    if(tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle) == 0)
        tmp_pos[0] = XMIN;
    else
        tmp_pos[0] = source_position[0] - (fabs(source_position[1] - tmp_pos[1])) / tan(sampleBuffer[SENSOR_NUM_MAX-1].instant_wind_angle);
    //printf("x %f\n", tmp_pos[0]);
    place_sensor(SENSOR_NUM_MAX-1, tmp_pos);

    // measure odor concentration
    double tmp_c = odor_read(SENSOR_NUM_MAX-1);
    //printf("sensor reading %f\n", tmp_c);
    if(tmp_c > 0)
        return false;
    else
        return true;
}

int main() {

    int i = 0;
    int wait = WAITING_TIME; // seconds
    double start_rec_time = 0;

    init();
  
    //Get the sensor values and write them in a file
    while(iteration < total_nbr_iteration){
        if(wb_robot_get_time() < wait){

           //wait for the plume to be established
           
            // if(still_wait() == false){
            //     wait = 0;    
            //     printf("Plume established!\n");
            //     place_sensor(SENSOR_NUM_MAX-1, sensor_position[SENSOR_NUM_MAX-1]);
            // }
        }
        else{
            // write instant concentration in sampleBuffer[i] (size = nb_iterations)
            if(iteration == 0){
                start_rec_time = wb_robot_get_time();
                printf("done waiting %d seconds !\n", wait);
            }
            
            for(i=0; i < sensor_number; i++){
                //continue measuring odor concentration
                sampleBuffer[i].instant_concentration = odor_read(i);
                //printf("odor sensor %d -> %f \n", i, sampleBuffer[i].instant_concentration);
                //printf("nb_samples (changed in samplebuffer) %d\n", sampleBuffer[i].n_samples);
                sampleBuffer[i].add_odor_to_buffer();
            }

            //LOG

            //read sensors' data and log
            char logfilename[50];
            sprintf(logfilename, "../../log/spx%.1f_spy%.1f_spz%.1f.csv", source_position[0], source_position[1], source_position[2]);
            // TODO: create the file if not exist, check the avaibility before write
            FILE * data_base = fopen(logfilename, "a");
            // if (!data_base){
               // printf("Error opening files in log\n");
               // continue;
            // }
            // print headers
            if(iteration == 0){
                fprintf(data_base, "# t (s),");
                i = 0;
                for(int z=0; z<(NB_SENSORS_Z); z++){
                    for(int x=0; x<(NB_SENSORS_X); x++){
                        for(int y=0; y<(NB_SENSORS_Y); y++){
                            i += 1;
                            if(i<sensor_number)
                                fprintf(data_base, "C_%d%d%d (ppm),", x, y, z);
                        }
                     }
                }      
                fprintf(data_base, "C_%d%d%d (ppm)\n", (NB_SENSORS_X-1), (NB_SENSORS_Y-1), (NB_SENSORS_Z-1));
            }
            //log in the data_base
            fprintf(data_base, "%f,", (wb_robot_get_time() - start_rec_time));
            for(i=0; i < (sensor_number-1); i++)
                fprintf(data_base, "%f,", sampleBuffer[i].instant_concentration);   // C
            fprintf(data_base, "%f\n", sampleBuffer[sensor_number-1].instant_concentration);
            fclose(data_base);
            
            printf("Iteration %d over\n", iteration+1);            
            iteration++;
            
        }
        //NEXT
        wb_robot_step(TIME_STEP);
    }
    // Calculate bouts frequency
    /*int bouts_frequency[sensor_number] = {0};
    for(i = 0; i<sensor_number; i++){
        compute_bouts(sensor_number);
    }*/
    
    //reset the buffer of sensors
    for (i=0; i<SENSOR_NUM_MAX; i++){
        sampleBuffer[i].reset_all();
    }
    
    printf("FINISHED all %d iterations)\n", total_nbr_iteration);
    //wb_supervisor_simulation_quit(EXIT_SUCCESS);
    wb_robot_cleanup();
    return 0;
}
