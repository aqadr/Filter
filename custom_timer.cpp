#include "custom_timer.h"

using namespace std;

custom_timer::custom_timer(){
    
      clock_gettime(CLOCK_REALTIME, &Base_time);
      abs_time=0;
    
  }

uint64_t custom_timer::get_current_time(){
  
  clock_gettime(CLOCK_REALTIME, &current_time);
  
  uint64_t time_diff=(current_time.tv_sec-Base_time.tv_sec)*1000000000+(current_time.tv_nsec-Base_time.tv_nsec);//*(1/1000);
//   cout<<current_time.tv_nsec<<endl;
//  cout<<"time_diff:"<<time_diff<<endl;
  abs_time +=time_diff;
  
//  Base_time = current_time;
  return time_diff;
}


void custom_timer::print_current_time(){
  
  printf("current time: %d\n",abs_time);
}


