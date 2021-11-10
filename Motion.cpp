#include<iostream>
#include <cmath>
#include"Motion.h"

using namespace std;
extern int Final_speed[8] ;

 void motion(float axis[6],int max_speed)
   {
       float Forward=axis[1];
       float Slide=axis[0];
       float Rotate=axis[2];
       float Pitch=axis[3];
       float Up=axis[4];
       float Roll=axis[5];

       int BFR=(Forward-Slide+Up-Rotate+Roll+Pitch)*max_speed;
       int BFL=(Forward+Slide+Up+Rotate-Roll+Pitch)*max_speed;
       int BBR=(Forward-Slide+Up+Rotate+Roll-Pitch)*max_speed;
       int BBL=(-Forward+Slide+Up-Rotate-Roll-Pitch)*max_speed;
       int TFR=(-Forward-Slide+Up+Rotate+Roll+Pitch)*max_speed;
       int TFL=(-Forward+Slide+Up-Rotate-Roll+Pitch)*max_speed;
       int TBR=(Forward-Slide+Up-Rotate-Roll-Pitch)*max_speed;
       int TBL=(Forward+Slide+Up+Rotate-Roll-Pitch)*max_speed;
       Final_speed[0]=BFR;
       Final_speed[1]=BFL;
       Final_speed[2]=BBR;
       Final_speed[3]=BBL;
       Final_speed[4]=TFR;
       Final_speed[5]=TFL;
       Final_speed[6]=TBR;
       Final_speed[7]=TBL;
       normalize();

}

   void normalize()
   {    float MAX =0;
        float scale;


       for(int i=0;i<8;i++)
       {
           if(abs(Final_speed[i])>MAX)
                MAX=abs(Final_speed[i]);
       }
        if(MAX>200)
        {
            scale=float(200.0/MAX);
            for(int i=0;i<8;i++)
                Final_speed[i]*=scale;

        }
   }
