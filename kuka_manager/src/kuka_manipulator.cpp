


//ros2 launch kuka_kr6_simulation kuka_position.launch.py


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/bool.h>
#include "kuka_manager/srv/move_arm.hpp"

#include <cmath>
#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;


#define N 6
#define M 4
#define pi 3.14159265358979323846

float envia_junta[N];
float q_atual[6];
float X_final[6];
int iter;
float t =0.0;

//--------------------------funções matmáticas--------------------------------
void getCofactor(float A[N][N], float temp[N][N], float p, float q, float n) {
    int i = 0, j = 0;

    for (int row = 0; row < n; row++) {
        for (int col = 0; col < n; col++) {

            if (row != p && col != q) {
                temp[i][j++] = A[row][col];

                if (j == n - 1) {
                    j = 0;
                    i++;
                }
            }
        }
    }
}

float determinant(float A[N][N], float n) {   //Calcula determinante
    float D = 0; // Initialize result

    if (n == 1)
        return A[0][0];

    float temp[N][N];

    float sign = 1;

    for (int f = 0; f < n; f++) {
        // Getting Cofactor of A[0][f]
        getCofactor(A, temp, 0, f, n);
        D += sign * A[0][f] * determinant(temp, n - 1);

        sign = -sign;
    }

    return D;
}

void adjoint(float A[N][N],float adj[N][N]) { // calcula matriz adjunta
    if (N == 1)
    {
        adj[0][0] = 1;
        return;
    }

    float sign = 1;
    float temp[N][N];

    for (int i=0; i<N; i++) {
        for (int j=0; j<N; j++) {

            getCofactor(A, temp, i, j, N);

            sign = ((i+j)%2==0)? 1: -1;

            adj[j][i] = (sign)*(determinant(temp, N-1));
        }
    }
}

void inv(float A[N][N],float inverse[N][N]) {  //Calcula matriz inversa
    // Find determinant of A[][]
    float det = determinant(A, N);
    if (det == 0) {
        cout << "Singular matrix, can't find its inverse";
    }

    float adj[N][N];
    adjoint(A, adj);

    // Find Inverse using formula "inverse(A) = adj(A)/det(A)"
    for (int i=0; i<N; i++)
        for (int j=0; j<N; j++)
            inverse[i][j] = adj[i][j]/float(det);
}

void multiplicaMatriz(float a[N][N], float b[N],float c[N]){
    float aux;
    for (int i=0; i<N; i++)
    {
            aux=0;
            for (int k=0; k<N; k++)
            {
                aux = aux + (a[i][k] * b[k]);

            }
            c[i]=aux;
    }
}

void multiplicaMatriz_quadrada(float a[M][M], float b[M][M], float c[M][M]){ //multiplica duas matrizes quadradas
    float aux;
    for(int i=0;i<M;i++){
        for(int j = 0;j<M; j++){
            aux = 0;
            for(int k =0; k<M; k++) aux = aux + (a[i][k]*b[k][j]);
            c[i][j] = aux; 
         }
    }

}
//---------------------Fim das funções matemáticas-----------------------------

//--------------------- Cinemática Direta -----------------------------

float * direta(float Q[N]){

    float theta, d, a, alpha;
    float A[4][4], B[4][4], S[4][4];

    double DH[6][4] = {{Q[0],400,25,-pi/2},{Q[1],0,315,0},{Q[2]+pi/2,0,-35,pi/2},{Q[3],365,0,-pi/2},{Q[4],0,0,pi/2},{Q[5],80,0,pi}};

    for (int i=0; i<6;i++){
    
        theta = DH[i][0];
        d = DH[i][1];
        a = DH[i][2];
        alpha = DH[i][3];

        float Rz[4][4] = {{cos(theta), -sin(theta), 0, 0},{sin(theta), cos(theta), 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};
        float Tz[4][4] = {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, d},{0, 0, 0, 1}};
        float Tx[4][4] = {{1, 0, 0, a},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}};
        float Rx[4][4] = {{1, 0, 0, 0},{0, cos(alpha), -sin(alpha), 0},{0, sin(alpha), cos(alpha), 0},{0, 0, 0, 1}};
    
        multiplicaMatriz_quadrada(Rz,Tz,A);
        multiplicaMatriz_quadrada(A,Tx,B);
        multiplicaMatriz_quadrada(B,Rx,A);

        if(i==0) {
            for(int j=0;j<4;j++){
                for(int k=0;k<4;k++) S[j][k] = A[j][k];
            }
        }
        else {
            multiplicaMatriz_quadrada(S,A,B);
            for(int j=0;j<4;j++){
                for(int k=0;k<4;k++) S[j][k] = B[j][k];
            }
        }
    }

    float x = S[0][3];
    float y = S[1][3];
    float z = S[2][3];
    float nx = S[0][0];
    float ny = S[1][0];
    float nz = S[2][0]; 
    float ox = S[0][1];
    float oy = S[1][1];
    float ax = S[0][2];
    float ay = S[1][2];

    float phi_r = atan2(ny,nx);
    float phi_p = atan2(-nz,(nx*cos(phi_r)+ny*sin(phi_r)));
    float phi_y = atan2((-ay*cos(phi_r)+ax*sin(phi_r)),(oy*cos(phi_r)-ox*sin(phi_r)));
    
    //float phi_r = atan2(-nz,sqrt(pow(nx,2)+pow(ny,2)));
    //float phi_p = atan2(ny/cos(phi_r),nx/cos(phi_r));
    //float phi_y = atan2(oz/cos(phi_r),az/cos(phi_r));

    static float pos[6];
    pos[0] = x;
    pos[1] = y;
    pos[2] = z;
    pos[3] = phi_r;
    pos[4] = phi_p;
    pos[5] = phi_y;

    return pos;
}

//---------------------Cinemática Inversa-----------------------------


void inversa(float pos_final[N], float theta[N]){
    float alpha = 0.005 , d = 1000, d_ant = 1001;
    float * aux;
    float inc_theta[N], Jt[N][N], jt[N][N], J_inv[N][N];
    float J_alpha[N][N],erro[N],delta_theta[N], pos_atual[N];
    
    float * a = direta(theta);
    for(int i=0;i<6;i++){
        pos_atual[i] = a[i];    
    }
    int m=0;
    while(sqrt((d_ant-d)*(d_ant-d))>0.000001 && m<100){
        for(int j=0;j<6;j++){
            for(int i=0;i<6;i++){
                if(i==j) inc_theta[i] = theta[i] + 0.01;
                else inc_theta[i] = theta[i];   
                
                if(i==5){
                    aux = direta(inc_theta);
                    for(int k=0;k<6;k++){
                        jt[j][k] = aux[k] - pos_atual[k];
                    }
                    
                }
            }
        }
        for(int i=0;i<6;i++){
            for(int j=0;j<6;j++){
                Jt[i][j] = jt[j][i];                  //Transpoe a matriz Jt
            }
        }
        inv(Jt,J_inv);
        for(int j=0;j<6;j++){				//inv(jt)*alpha
           for(int i=0;i<6;i++){
                J_alpha[i][j] = J_inv[i][j]*alpha;
           }
           erro[j] = pos_final[j]- pos_atual[j];
        }
        multiplicaMatriz(J_alpha,erro,delta_theta);
        for(int j=0;j<6;j++){//theta
                theta[j] = theta[j]+delta_theta[j];
        }
        a = direta(theta);
        for(int i=0;i<6;i++){
            pos_atual[i] = a[i];    
        }
        d_ant = d;
        d = sqrt(pow((pos_final[0]- pos_atual[0]),2)+pow((pos_final[1]- pos_atual[1]),2)+pow((pos_final[2]- pos_atual[2]),2));
	m++;
    
    
    }
    
    for(int k=0;k<6;k++){
        envia_junta[k] = theta[k];
        if(envia_junta[k]< -pi ||envia_junta[k]>pi){
            while (envia_junta[k] <= -pi) {
                envia_junta[k] += 2*pi;
            }
            while (envia_junta[k] > pi) {
                envia_junta[k] -= 2*pi;
            }
        }
    }
    iter = m;
   
    
}

class robo : public rclcpp::Node
{
public:
  robo()
  : Node("robo")
  {

    auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
     
    service = this->create_service<kuka_manager::srv::MoveArm>("move_arm", std::bind(&robo::MoveArm, this, _1,_2)); 
     
    joint_states_subscription_= this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, std::bind(&robo::topic_callback_joint, this, _1));
      
      
    publisher_= this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_command_controller_position/commands", default_qos); 
 
  }

private:
string mensagem;

void MoveArm(std::shared_ptr<kuka_manager::srv::MoveArm::Request> request,    
          std::shared_ptr<kuka_manager::srv::MoveArm::Response>  response)  
{     
	   
          X_final[0] =  request->x;  
          X_final[1] =  request->y;  
          X_final[2] =  request->z;  
          X_final[3] =  request->roll;  
          X_final[4] =  request->pitch;     
          X_final[5] =  request->yaw;      
                
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parâmetros recebidos\nx: %f" " y: %f" " z: %f" " roll: %f" " pitch: %f" " yaw: %f",  
                request->x, request->y, request->z, request->roll, request->pitch, request->yaw );                  
                                      
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Enviando resposta: [%f]", response->succes);
	  
  // ros2 service call /move_arm kuka_manager/srv/MoveArm "{x: 100, y: 100, z: 950, roll: 0, pitch: 0, yaw: 0}"


         //Cálculo da cinemática inversa          
          inversa(X_final, q_atual); 
          cout << "..." << endl;
          
          
	  //Publica os valores das juntas 
	   vector<double> msgdata;  
	   msgdata.push_back(envia_junta[0]);
	   msgdata.push_back(envia_junta[1]);
	   msgdata.push_back(envia_junta[2]);
	   msgdata.push_back(envia_junta[3]);
	   msgdata.push_back(envia_junta[4]);
	   msgdata.push_back(envia_junta[5]);
	   
	   std_msgs::msg::Float64MultiArray msg1;
 
	   for(long unsigned int itr = 0; itr != msgdata.size(); itr++) {
		msg1.data.push_back(msgdata[itr]); 
	    }
	    
	    publisher_->publish(std::move(msg1)); 
  
  
}


void topic_callback_joint(const sensor_msgs::msg::JointState::SharedPtr msg) const
  {    
      
    double theta6 = msg->position[0];
    double theta1 = msg->position[1];
    double theta2 = msg->position[2];
    double theta3 = msg->position[3];
    double theta5 = msg->position[4];
    double theta4 = msg->position[5];
    
    q_atual[0]=theta1;
    q_atual[1]=theta2;
    q_atual[2]=theta3;
    q_atual[3]=theta4;
    q_atual[4]=theta5;
    q_atual[5]=theta6;
	  
          /* cout << "Insira a coordenada de x: " ;
	   cin >> X_final[0];
	   cout << "Insira a coordenada de y: " ;
	   cin >> X_final[1];
	   cout << "Insira a coordenada de z:  " ;
	   cin >> X_final[2];
	   cout << "Insira o valor de roll:  " ;
	   cin >> X_final[3];
	   cout << "Insira o valor de pitch: " ;
	   cin >> X_final[4];
	   cout << "Insira o valor de yaw: " ;
	   cin >> X_final[5]; 

	// Cálculo cinemática inversa
	 inversa(X_final, q_atual);
	   cout << "..." << endl;*/
   
  }
  
  // Declara subscription, publisher e service  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  rclcpp::Service<kuka_manager::srv::MoveArm>::SharedPtr service;
  
};


int main(int argc, char **argv)
{
  rclcpp::init(argc, argv); 

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pronto para receber paramêtros");
  
  rclcpp::spin(std::make_shared<robo>());

  rclcpp::shutdown();
  return 0;
}




























