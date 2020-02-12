#include <iostream>
#include <cmath>
#include <math.h>
#include <vector>
using namespace std;



int main() {
    
    //initial positions, state 0
    int x = 2; //original x-coordinate
    int y = 2; //original y-coordinate
    float theta = 90; //original rotation angle
    vector<double> state;
    state.push_back(0);
    state.push_back(0);
    state.push_back(0);
    state.push_back(0);
    
    //new positions
    int new_x = 0; //x'
    int new_y = 0; //y'
    float new_theta; //theta'
    
    //constants
    double h = 2; //distance travelled by robot in one f/b pulse
    double phi = 5; //angle rotated by robot in one l/r pulse
    double t_interval = 0.08; //time interval in seconds used to calculate rotational speed
    double L = 1000; //known distance between origin and right wall
    double W = 1000; //known distance between origin and front wall
    
    //new constants
    double v_r = 2; //translational right velocity
    double v_l = 2; //translational left velocity
    double v = 0; //translational velocity
    double delta_theta = 0; //change in rotational velocity
    double d = 25.4; //half of the width of the car
    double R;
    float delta_x;
    float delta_y;
    double d1, d2, d3, d4;
    double l1, l2, l3, l4;
    
    bool running = true;
    string input = "";
    
    while (running == true)
    {
        cout << "Enter the input: ";
        cin >> input;
        
        /*      SYSTEM DYNAMICS MODELS      */
        
        //forward pulse
        if (input == "F")
        {
            //new_x = x + h * cos(theta);
            //new_y = y + h * sin(theta);
            new_theta = theta;
            
            //new
            v_r = 2;
            v_l = 2;
            delta_theta = 0;
        }
        
        //backward pulse
        else if (input == "B")
        {
            //new_x = x - h * cos(theta);
            //new_y = y - h * sin(theta);
            new_theta = theta;
            
            //new
            v_r = -2;
            v_l = -2;
            delta_theta = 0;
        }
        
        //leftward pulse
        else if (input == "L")
        {
            //new_x = x;
            //new_y = y;
            
            //new
            v_r = 2;
            v_l = -2;
            delta_theta = 4 / 2*d;
            new_theta = theta + delta_theta;
        }
        
        //rightward pulse
        else if (input == "R")
        {
            //new_x = x;
            //new_y = y;
            
            //new
            v_r = -2;
            v_l = 2;
            delta_theta = -4 / 2*d;
            new_theta = theta + delta_theta;
        }
        
        //none, end simulation
        else
        {
            running = false;
            break;
        }
        
        /*      SENSOR MEASUREMENT MODELS      */
        int d_f = W - new_y; //calculate new distance b/w robot and front wall
        int d_r = L - new_x; //calculate new distance b/w robot and right wall
        int rot_sp = (new_theta - theta) / t_interval; //calculate rotational speed
        
        
       // calculations
        
        // mathematical model
        v = (v_r + v_l) / 2;
        R = (2*d*v_r) / (v_l - v_r);
        delta_x = v * cos(new_theta * M_PI / 180.0);
        delta_y = v * sin(new_theta * M_PI / 180.0);
        new_x = x + delta_x;
        new_y = y + delta_y;
        
        // measurement model
        d1 = abs((L - new_x)/cos(new_theta * M_PI / 180.0));
        d2 = abs((W - new_y)/sin(new_theta * M_PI / 180.0));
        d3 = abs((new_x)/cos((new_theta+180.0)* M_PI / 180.0));
        d4 = abs((new_y)/sin((new_theta+180.0)* M_PI / 180.0));
        
        l1 = abs((L-new_x)/sin(new_theta * M_PI / 180.0));
        l2 = abs((W-new_y)/cos((new_theta+180.0)* M_PI / 180.0));
        l3 = abs((new_x)/sin((new_theta+180.0)* M_PI / 180.0));
        l4 = abs((new_y)/cos(new_theta* M_PI / 180.0));
        
        state[0] = new_x;
        state[1] = new_y;
        state[2] = new_theta;
        state[3] = rot_sp;
        
        
        //output new state and sensor measurements
        cout << "new_theta = " << new_theta << endl;
        cout << "delta x = " << delta_x << endl;
        cout << "Current position: (" << new_x << ", " << new_y;
        cout << "), Current rotation angle: " << new_theta << endl;
        cout << "New front wall distance: " << d_f;
        cout << ", New right wall distance: " << d_r;
        cout << ", Rotational speed: " << rot_sp;
        cout << ", Translational velocity: " << v;
        cout << ", Distance R: " << R << endl;
        cout << "d1 = " << d1 << ", d2 = " << d2 << ", d3 = " << d3 << ", d4 = " << d4 << endl;
        cout << "l1 = " << l1 << ", l2 = " << l2 << ", l3 = " << l3 << ", l4 = " << l4 << endl;
        cout << "state: " << state[0] << " " << state[1] << " " << state[2] << " " << state[3] << endl;
        cout << endl;
        
        //update positions
        x = new_x;
        y = new_y;
        theta = new_theta;
    }
    
}
