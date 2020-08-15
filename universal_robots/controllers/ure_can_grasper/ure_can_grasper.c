/*
 * Copyright 1996-2020 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>
#include <math.h>

#include <stdio.h>

#define TIME_STEP 32

//calcula a matriz DH e seus parametos
void multiplicarMatriz(double mat1[4][4], double mat2[4][4], double mat3[4][4], double pos[4],double saida[4]){

 double matAux[4][4] = {
 {0,0,0,0}
 ,{0,0,0,0}
 ,{0,0,0,0}
 ,{0,0,0,0}};
 
 double matAux2[4][4] = {
 {0,0,0,0}
 ,{0,0,0,0}
 ,{0,0,0,0}
 ,{0,0,0,0}};
 
 for(int i=0;i<4;i++){
 
   for(int j=0;j<4;j++){
   
     for(int k=0;k<4;k++){
     
         matAux[i][j] += mat1[i][k]*mat2[k][j]; 
     
     }
   
   }
 
 }
 for(int i=0;i<4;i++){
 
   for(int j=0;j<4;j++){
   
     for(int k=0;k<4;k++){
     
         matAux2[i][j] += matAux[i][k]*mat3[k][j]; 
     
     }
   
   }
 
 }
 
 for(int i=0;i<4;i++){
 
   for(int j=0;j<4;j++){
   
         //aqui é o vetor de saida da posicao final apos a multiplicacao de matriz
         saida[i] += matAux2[i][j]*pos[j]; 
     
   }
 
 }


}

int main(int argc, char **argv) {
  wb_robot_init();
  int i = 0;
  //aqui são os angulos escolhidos para rotaao do robo
  double teta1=0,teta2=0.5,teta3=0.5;
  //double teta1=0,teta2=-1.88,teta3=-2.14;
  //theta[] = {0, -1.88, -2.14};
  //aqui controla a otacao final de cada eixo
  const double target_positions[] = {teta1, teta2, teta3};
  double speed = 1.0;
  //ini = {-0.226, 0.671, 1.284, 1}
  
  //aqui são as matrizes de DH
  double eixo1[4][4]={
  {cos(teta1),0,sin(teta1),0},
  {sin(teta1),0,-cos(teta1),0},
  {0,1,0,0},
  {0,0,0,1}};
  
  double eixo2[4][4]={
  {cos(teta2),-sin(teta2),0,0.3*cos(teta2)}
  ,{sin(teta2),cos(teta2),0,0.3*sin(teta2)}
  ,{0,0,1,0}
  ,{0,0,0,1}}; 
  
  double eixo3[4][4]={
  {cos(teta3),-sin(teta3),0,0.3*cos(teta3)}
  ,{sin(teta3),cos(teta3),0,0.3*sin(teta3)}
  ,{0,0,1,0}
  ,{0,0,0,1}};
  
  //essa é a posição fixada garra
  double position[] = {0.273928,0.23533,0.88603,1};
  // ini = {-0.226, 0.671, 1.284, 1}
  //double position[] = {-0.226, 0.671, 1.284, 1};
  double saida [] = {0,0,0,0};
  multiplicarMatriz(eixo1,eixo2,eixo3,position,saida);
  
  if (argc == 2)
    sscanf(argv[1], "%lf", &speed);

  //inicializa os motores e os dispositivos
  WbDeviceTag hand_motors[3];
  hand_motors[0] = wb_robot_get_device("finger_1_joint_1");
  hand_motors[1] = wb_robot_get_device("finger_2_joint_1");
  hand_motors[2] = wb_robot_get_device("finger_middle_joint_1");
  WbDeviceTag ur_motors[4];
  ur_motors[0] = wb_robot_get_device("shoulder_pan_joint");
  ur_motors[1] = wb_robot_get_device("shoulder_lift_joint");
  ur_motors[2] = wb_robot_get_device("elbow_joint");
  ur_motors[3] = wb_robot_get_device("wrist_1_joint");
  
  //seta uma velocidade para cada motor
  for (i = 0; i < 4; i++)
    wb_motor_set_velocity(ur_motors[i], speed);

  //faz a rotacao necessaria para cada motor
   for (i = 0; i < 3; i++)
              wb_motor_set_position(ur_motors[i], target_positions[i]);

   //mostra a saida
   printf("x: %f y: %f z: %f\n",saida[0],saida[1],saida[2]);

  wb_robot_cleanup();
  return 0;
}
