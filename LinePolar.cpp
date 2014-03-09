#include "LinePolar.h"
#include <string.h>
#include <algorithm>

    /**
     * function that gets the latest lidar data and stores them in the vector latest data
     */
    void LinePolar::UpdateData(struct lidar_data *scan_data,float theta_res) {
      
	latest_data->resize(0);
	_lines.resize(0);
	unsigned int data;
	char *ptr=scan_data->ptr;
	int data_size = strlen(scan_data->ptr);
        size_t indx;
	char buff[10];
	float theta=0;
	int cnt=0;
	struct scan_pt pt; 
	
	for (int i =0; i<data_size; i++) {
      
	    if (ptr[i]=='\t') {
	  
		for(int j=indx;j<i;j++){
		    buff[j-indx]=ptr[j];
		}
		data=atoi(buff);
		pt.dist=data;
		pt.theta=theta;
		pt.index=cnt;
		latest_data->push_back(pt);
		cnt++;
		theta=theta+theta_res;
		indx=i+1;
	    }
	}
    }
      
      
      
    /** 
     * function to compute a line from two laser scan points
     * and store the line parameters (r, alpha) in an array called line
     * this function will be used in the PolarLineFit function
     */
    void LinePolar::polar_line (struct scan_pt p_1, struct scan_pt p_2, float line[2]){
      
        float dt = sin(p_2.theta-p_1.theta);
	float sigma=(dt>0)?1:((dt<0)?-1:0);				// implementation of the sgn(signum in ternary operator)
	
	float deno=sqrt(p_1.dist*p_1.dist+p_2.dist*p_2.dist-2*p_1.dist*p_2.dist*cos(p_2.theta-p_1.theta));
	line[0]=sigma*p_1.dist*p_2.dist*sin(p_2.theta-p_1.theta)/deno;
	line[1]=acos(sigma*(p_2.dist*sin(p_2.theta)-p_1.dist*sin(p_1.theta))/deno);
    }
	
    
    /**
     * compute the score for the line
     * param@last_data: 	latest laser scan data
     * param@line: 		array that contains the line parameters(dist,alpha)
     * param@dist_thresh:	threshold for the points to be near the line
     */
    int LinePolar::score(const std::vector<scan_pt> last_data, const float line[2], float dist_thresh) {
      
        int points_on_line=0;
	scan_pt pt;
	int size = last_data.size();
	for (int i =0; i<size; i++) {
	    
	      pt=last_data.at(i);
	      float dist= line_dist(line, pt);
	      if (dist<dist_thresh) {
		points_on_line++;
	      }
	}
	
	return points_on_line;
    }
	
	
	
    /**
     * sorting algorithm
     */
    
    bool Lg_to_sm( const line_param &i, const line_param &j) {
      
	return (i.score > j.score);
    }
    
    bool Sm_to_Lg( const line_param &i, const line_param &j) {
      
	return (i.score < j.score);
    }
    
    /**
     * condition for removal the lines that do not have enough votes
     */

    template<int THRESH>
    
    bool sl_limit(const line_param &l) {
      
	return (l.score < THRESH);
    }
    
    
   
    
    
    
	
    /**
     * function that generates the line parameters and store them in a vector
     * each line contains the line parameters (dist, alpha) and a score (how many points are within the threshold
     * 
     */
    std::vector<struct line_param> LinePolar::PolarLineFit(float p_dist_threshold) {
      
	 std::vector<scan_pt> points;				//temporary hold all the scan points
	 float line[2];
	 std::vector<scan_pt> pts_on_line;
	 for (int i=0;i<latest_data->size();i++) {
	    points.push_back(latest_data->at(i));
	 }
	 
	int idx[]={0,0};
	int pts_size=latest_data->size();
	
	std::vector<line_param> score_line;			// a container that holds the lines created and also the score for each line
	line_param l;
	
	while (score_line.size()<_num_attemps) {
	  
	    /** grab two points*/
	    idx[0]= rand()%(int)points.size();
	    idx[1]= rand()%(int)points.size();
	    
	    /** if the points are the same, we do not have a line*/
	    if(idx[0] == idx[1]) continue;
	    
	    polar_line (points.at(idx[0]), points.at(idx[1]), line);
	    int points_on_line=score(points, line, p_dist_threshold);
	    
	    l.r=line[0];
	    l.alpha=line[1];
	    l.score=points_on_line;
	    score_line.push_back(l);
	}
	
	scan_pt p_1;
	scan_pt p_2;
	int pt_index;
	
	
	while(score_line.size() > 0) {
	  
	    /** sort the lines to get the one with the most votes to the top */
	    sort(score_line.begin(), score_line.end(), Sm_to_Lg);
	    std::cout<<"\n*******************************************"<<std::endl;
	    std::cout<<" the next best line is "<<score_line.back().r<<score_line.back().alpha<<"  with score "<<score_line.back().score<<std::endl;
	    
	    if (score_line.back().score <=(int)_point_thresh) {
	      
		std::cout<<" There are no lines long enough to be inportant. EXITING!" <<std::endl;
		break;
	    }
	    
	    
	    //place best line into a new vector and record the closest points here only the points that are adjacent to each other are stored 
	    // in the new class PointFitLine 
	    _lines.push_back(PointFitLine(score_line.back().score,score_line.back()) );
	    
	    line[0]=score_line.back().r;
	    line[1]=score_line.back().alpha;
	    
	    std::vector<scan_pt> tmp;
	    
	    for (int i =0; i<points.size(); i++) {
	    
		scan_pt pt=points.at(i);
		float dist= line_dist(line, pt);

		if (dist<p_dist_threshold) {
		    tmp.push_back(pt);
		}
	    }
	    
	    // algorithm that stores the points near the line. It searches for points that are sequential and store only the points that 
	    // are sequential
	    p_1 = tmp.front();
	    
	    for (int j=1; j<tmp.size(); j++) {
		
		if (tmp.at(j).index-tmp.at(j-1).index>4) {
		    p_2=tmp.at(j-1);
		    pt_index = j-1;
		    break;
		    
		} else {
		  
		    p_2=tmp.back();
		    pt_index = tmp.size();
		}
	    }
	    
	    for (int i = p_1.index;i<p_2.index;i++) {
		pts_on_line.push_back(points.at(i));
	    }
	       
	    _lines.back().points_on_line=pts_on_line;
	    
	    // removes the top score line
	    score_line.pop_back();
	    
	    // removing the points from the points vecotr
	    // remove all points belonging to best line
	    typename std::vector<scan_pt>::iterator pntend = remove_if(points.begin(), points.end(),in_limit(p_1.index, p_2.index));
	    
	    
	    points.resize(std::distance(points.begin(), pntend) );
	    std::cout<<" There are now "<<points.size() << "points" <<std::endl;
	    
	    // rescore all lines 
	    for (int i=0; i<score_line.size(); i++) {
		line[0]=score_line.at(i).r;
		line[1]=score_line.at(i).alpha;
		int points_on_line=score(points, line, p_dist_threshold);
	    	score_line.at(i).score=points_on_line;
	    }
	    
	    // Find all lines with not enough votes to live 
	    std::vector<line_param>::iterator sl_it = remove_if(score_line.begin(), score_line.end(), sl_limit<8>);
	    
	    // Resize the vector to remove unwanted lines
	    score_line.resize(std::distance(score_line.begin(), sl_it));
	    
	    
	    if (score_line.size() ==0)
		std::cout << " There are not enough lines. EXITING!" <<std::endl;
	    
	}
	
    }
	
	
	
	
    