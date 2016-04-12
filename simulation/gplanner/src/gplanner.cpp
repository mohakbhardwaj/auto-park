#include <gplanner.h>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>
#include <iterator>


using namespace std;

#define INF 1000000

globalPlanner::globalPlanner()
{
	params.wexit=1;
    params.wentry=1;
    params.woccupied=1;
    params.wqueue=1;
    params.wtime=1;

    nofSpots=104;
    initCosts();
    


}


void globalPlanner::getQuery(int qval)
{
	//cout<<qval;
    qSize=qval;
    calculateFinalCosts();

}


void globalPlanner::startD2Exitplanner()
{
    if(pp->plan())
    {
        exitSpotCosts=pp->getSpotCosts();
    }
}


void globalPlanner::initCosts()
{
    for (int i=0;i<nofSpots;i++)
    {
        exitSpotCosts.push_back(nofSpots-i);
        finalSpotCosts.push_back(0);
    }
}

void globalPlanner::getTimeCosts()
{
}

void globalPlanner::calculateFinalCosts()

{   
    int sum=0;
    for (std::vector<double>::iterator it =exitSpotCosts.begin(); it!=exitSpotCosts.end(); ++it)
    {
        sum+=*it;
    }
    
    for (int i=0; i< nofSpots;i++)
    {
        //if(state[i]==1)
        // {
        //     finalSpotCosts[i]=INF;
        // }
        // else
        // {
            //finalSpotCosts[i]= params.wexit*normExitCosts[i]+ params.wqueue*qSize*i/1000 +
            finalSpotCosts[i]= params.wexit*exitSpotCosts[i]/sum+ params.wqueue*qSize*i/1000 +          
            params.wtime * i;
        // }
    }

    // write code to find minimum from best five spots and query local planner based on that
//launch-prefix="xterm -e gdb"

}


void openFile(std::ifstream& file, std::string filename)
{
    while(true)
    {
        file.open(filename.c_str());
        if(file.is_open())
        {
            break;
        }
        else
        {
            std::cerr<<"No such file exists with name "<< filename << std::endl;
            file.clear();
            // ros::Duration(0.1).sleep(); //sleep coz blaaahhhhh
        } 

    }
}

void globalPlanner::useCache()
{   
    double num=0;
   
    std::ifstream costFile;
    std::string s;
    openFile(costFile,"/home/shivam/catkin_ws/costs.txt");
    //("costs.txt", std::ios::in);
    //costFile.clear();
    //costFile.seekg(0, ios::beg);
    cout<<"Cached  Costs"<<endl;
    int i=0;
    while(getline(costFile,s))
    {   
        std::stringstream ss(s);
        double num;     
        ss>>num;
        exitSpotCosts[i]=num;
        i++;

        cout<<exitSpotCosts[i]<<endl;
    }

    // for(int i =0; i<(exitSpotCosts.size()); i++)
    // {   
    //     costFile>>num;
    //     exitSpotCosts[i]=num;
    //     cout<<i<<" "<<exitSpotCosts[i]<<endl;
    //     if (costFile.eof())
    //     {
    //         while(i<exitSpotCosts.size())
    //         {
    //             exitSpotCosts[i]=double(num);
    //             cout<<i<<" "<<num<<" "<<exitSpotCosts[i]<<endl;
    //             i++;
    //         }
    //         break;
    //     }

    // }

}


int globalPlanner::returnFinalSpot()
{
    auto spot =std::min_element(std::begin(finalSpotCosts),std::end(finalSpotCosts));
    
    int i= std::distance(std::begin(finalSpotCosts),spot);

    exitSpotCosts[i]=INF;
    return i;

}

std::vector<double> globalPlanner::normalize(std::vector<double> v)
{   
    double sum=0;
    for (std::vector<double>::iterator it =v.begin(); it!=v.end(); ++it)
    {
        sum+=*it;
    }

    for (std::vector<double>::iterator it =v.begin(); it!=v.end(); ++it)
    {
        *it=*it/sum;
    }

}

envState globalPlanner::returnConfig(int i)
{
    struct envState e=pp->spotIDtoCoord(i);
    cout<<" GOALS SENT "<<e.x<<" "<<e.y<<" "<<e.y;
    return e;
}
/*
void PoseStampedtoPose(const geometry_msgs::PoseStamped& p1, Pose& p2)
{
    p2.x =  p1.pose.position.x;
    p2.y = p1.pose.position.y;
    tf::Quaternion p1_quat = tf::Quaternion(p1.pose.orientation.x,p1.pose.orientation.y,
                                            p1.pose.orientation.z, p1.pose.orientation.w);
    p2.th = tf::getYaw(p1_quat);

}
void PosetoPoseStamped(geometry_msgs::PoseStamped& p1, const Pose& p2)
{
    
    tf::Quaternion p1_quat = tf::createQuaternionFromYaw(p2.th);
            
    p1.pose.position.x = p2.x;
    p1.pose.position.y = p2.y;
              
    p1.pose.orientation.x = p1_quat.x();
    p1.pose.orientation.y = p1_quat.y();
    p1.pose.orientation.z = p1_quat.z();
    p1.pose.orientation.w = p1_quat.w();

}
*/