#ifndef CONFIG_H
#define CONFIG_H

// Data structure to hold info for a single servo
struct servoStruct {
    int min;
    int mid;
    int max;
};

const servoStruct s_torso = {500, 1500, 2500};   // TORSO
const servoStruct s_ra1 =   {500, 1500, 2500};   // RIGHT ARM SHOULDER BASE
const servoStruct s_ra2 =   {500, 1500, 2500};   // RIGHT ARM SHOULDER SWING
const servoStruct s_ra3 =   {500, 1500, 2500};    // RIGHT ARM BICEP SWIVEL

const servoStruct s_rl1 =   {500, 1500, 2500};    // RIGHT LEG THIGH
const servoStruct s_rl2 =   {500, 1500, 2500};    // RIGHT LEG FEMUR
const servoStruct s_rl3 =   {500, 1500, 2500};    // RIGHT LEG TIBIA
const servoStruct s_rl4 =   {500, 1500, 2500};    // RIGHT LEG ANKLE

const servoStruct s_ll1 =   {500, 1500, 2500};    // LEFT LEG THIGH
const servoStruct s_ll2 =   {500, 1500, 2500};    // LEFT LEG FEMUR
const servoStruct s_ll3 =   {500, 1500, 2500};    // LEFT LEG TIBIA
const servoStruct s_ll4 =   {500, 1500, 2500};    // LEFT LEG ANKLE

const servoStruct s_la1 =   {500, 1500, 2500};   // LEFT ARM SHOULDER BASE
const servoStruct s_la2 =   {500, 1500, 2500};   // LEFT ARM SHOULDER SWING
const servoStruct s_la3 =   {500, 1500, 2500};    // LEFT ARM BICEP SWIVEL

const servoStruct servoCluster[] = {
  s_torso, s_ra1, s_ra2, s_ra3,
  s_rl1, s_rl2, s_rl3, s_rl4,
  s_ll1, s_ll2, s_ll3, s_ll4,
  s_la1, s_la2, s_la3
};

#endif