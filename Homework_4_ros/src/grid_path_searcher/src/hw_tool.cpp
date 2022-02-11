#include <hw_tool.h>

using namespace std;
using namespace Eigen;

void Homeworktool::initGridMap(double _resolution, Vector3d global_xyz_l, Vector3d global_xyz_u, int max_x_id, int max_y_id, int max_z_id)
{   
    gl_xl = global_xyz_l(0);
    gl_yl = global_xyz_l(1);
    gl_zl = global_xyz_l(2);

    gl_xu = global_xyz_u(0);
    gl_yu = global_xyz_u(1);
    gl_zu = global_xyz_u(2);
    
    GLX_SIZE = max_x_id;
    GLY_SIZE = max_y_id;
    GLZ_SIZE = max_z_id;
    GLYZ_SIZE  = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE = GLX_SIZE * GLYZ_SIZE;

    resolution = _resolution;
    inv_resolution = 1.0 / _resolution;    

    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
}

void Homeworktool::setObs(const double coord_x, const double coord_y, const double coord_z)
{   
    if( coord_x < gl_xl  || coord_y < gl_yl  || coord_z <  gl_zl || 
        coord_x >= gl_xu || coord_y >= gl_yu || coord_z >= gl_zu )
        return;

    int idx_x = static_cast<int>( (coord_x - gl_xl) * inv_resolution);
    int idx_y = static_cast<int>( (coord_y - gl_yl) * inv_resolution);
    int idx_z = static_cast<int>( (coord_z - gl_zl) * inv_resolution);      
    
    data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] = 1;
}

bool Homeworktool::isObsFree(const double coord_x, const double coord_y, const double coord_z)
{
    Vector3d pt;
    Vector3i idx;
    
    pt(0) = coord_x;
    pt(1) = coord_y;
    pt(2) = coord_z;
    idx = coord2gridIndex(pt);

    int idx_x = idx(0);
    int idx_y = idx(1);
    int idx_z = idx(2);

    return (idx_x >= 0 && idx_x < GLX_SIZE && idx_y >= 0 && idx_y < GLY_SIZE && idx_z >= 0 && idx_z < GLZ_SIZE && 
           (data[idx_x * GLYZ_SIZE + idx_y * GLZ_SIZE + idx_z] < 1));
}

Vector3d Homeworktool::gridIndex2coord(const Vector3i & index) 
{
    Vector3d pt;

    pt(0) = ((double)index(0) + 0.5) * resolution + gl_xl;
    pt(1) = ((double)index(1) + 0.5) * resolution + gl_yl;
    pt(2) = ((double)index(2) + 0.5) * resolution + gl_zl;

    return pt;
}

Vector3i Homeworktool::coord2gridIndex(const Vector3d & pt) 
{
    Vector3i idx;
    idx <<  min( max( int( (pt(0) - gl_xl) * inv_resolution), 0), GLX_SIZE - 1),
            min( max( int( (pt(1) - gl_yl) * inv_resolution), 0), GLY_SIZE - 1),
            min( max( int( (pt(2) - gl_zl) * inv_resolution), 0), GLZ_SIZE - 1);                  
  
    return idx;
}

Eigen::Vector3d Homeworktool::coordRounding(const Eigen::Vector3d & coord)
{
    return gridIndex2coord(coord2gridIndex(coord));
}

double Homeworktool::OptimalBVP(Eigen::Vector3d _start_position,Eigen::Vector3d _start_velocity,Eigen::Vector3d _target_position)
{
    //double optimal_cost = 100000; // this just to initial the optimal_cost, you can delete it 
    /*
    STEP 2: go to the hw_tool.cpp and finish the function Homeworktool::OptimalBVP
    the solving process has been given in the document

    because the final point of trajectory is the start point of OBVP, so we input the pos,vel to the OBVP

    after finish Homeworktool::OptimalBVP, the Trajctory_Cost will record the optimal cost of this trajectory
    */
    double optimal_cost = 100000.0; 
    double deltaPx = _target_position(0) - _start_position(0);
    double deltaPy = _target_position(1) - _start_position(1);
    double deltaPz = _target_position(2) - _start_position(2);
    double deltaVx = 0 - _start_velocity(0);
    double deltaVy = 0 - _start_velocity(1);
    double deltaVz = 0 - _start_velocity(2);
    double Vx0 = _start_velocity(0);
    double Vy0 = _start_velocity(1);
    double Vz0 = _start_velocity(2);

    double m = deltaPx*deltaPx +deltaPy*deltaPy + deltaPz*deltaPz;
    double n = 2*(deltaPx*Vx0 + deltaPy*Vy0 + deltaPz*Vz0) + 
                  deltaPx*deltaVx + deltaPy*deltaVy + deltaPz*deltaVz;
    double k = 3*(Vx0*Vx0 + Vy0*Vy0 +Vz0*Vz0 + deltaVx*Vx0 + deltaVy*Vy0 + deltaVz*Vz0) +
                  deltaVx*deltaVx + deltaVy*deltaVy + deltaVz*deltaVz;
    
    // Eigen库相关使用资料https://blog.csdn.net/shuzfan/article/details/52367329
    Eigen::Matrix<double, 4, 4> CompanionMatrix44;
    Eigen::Matrix<complex<double>, Eigen::Dynamic, Eigen::Dynamic> CompanionMatrix44EigenValues;//复数动态矩阵

    double c = - 4*k;
    double d = - 24*n;
    double e = - 36*m;
    vector<double> tmpOptimalT(4 ,0.0);
    vector<double> tmpOptimalCost(4, 100000.0);
    double optimalT = 0.0;
    CompanionMatrix44 << 0, 0, 0, -e,
				         1, 0, 0, -d,
				         0, 1, 0, -c,
				         0, 0, 1, 0 ;

    //std::cout<<"CompanionMatrix44: "<<std::endl<<CompanionMatrix44<<std::endl<<std::endl;                     
    CompanionMatrix44EigenValues = CompanionMatrix44.eigenvalues();
    //std::cout<<"matrix_eigenvalues: "<<std::endl<<CompanionMatrix44EigenValues<<std::endl;

    cout << "---------------" << endl;
    cout << CompanionMatrix44EigenValues << endl;

    for(int i = 0; i < CompanionMatrix44EigenValues.size(); i++)
    {
    	//TODO:时间不能为负数
        // 判断条件怎么来的？imag() == 0，剔除特征值为复数的情况，real() > 0，时间不能为负数
        // ignoring negative roots and complex roots, if all roots are complex, the function J is monotonous
        if(CompanionMatrix44EigenValues(i).imag() == 0  && CompanionMatrix44EigenValues(i).real() > 0)
        {
            // 为什么特征值可以直接拿过来当解？参考：https://blog.csdn.net/fb_941219/article/details/102984587
            tmpOptimalT[i] = CompanionMatrix44EigenValues(i).real();
            double t =  tmpOptimalT[i];
            double t2 = t*t;
            double t3 = t2*t;
            tmpOptimalCost[i] = t + (12*m)/(t3) + (12*n)/(t2) + (4*k)/t;
        }
        else
        {
            continue;
        }
    }

    // for(int i = 0; i < tmpOptimalCost.size(); i++)
    // {
    //     optimal_cost = std::min(tmpOptimalCost[i],optimal_cost);
    // }
    // for(vector<double>::iterator it = tmpOptimalCost.begin(); it != tmpOptimalCost.end(); it++)
    for(auto it = tmpOptimalCost.begin(); it != tmpOptimalCost.end(); it++)
    {
        cout << *it << endl;
    }
    cout << "---------------" << endl;
    optimal_cost = *min_element(tmpOptimalCost.begin(), tmpOptimalCost.end());
	//可以不用
    int flag = 0;
    for(int i = 0; i < tmpOptimalCost.size(); i++)
    {
        if(optimal_cost == tmpOptimalCost[i])
        flag = i;
        else continue;
    }
    optimalT = tmpOptimalT[flag];
    return optimal_cost;
}
