#include <iostream>
#include <cmath>
using namespace std;

int main() {
    
    //initial positions, state 0
    int x = 0; //original x-coordinate
    int y = 0; //original y-coordinate
    int theta = 90; //original rotation angle
    
    //new positions
    int new_x; //x'
    int new_y; //y'
    int new_theta; //theta'
    
    //constants
    int h = 2; //distance travelled by robot in one f/b pulse
    int phi = 5; //angle rotated by robot in one l/r pulse
    float t_interval = 0.08; //time interval in seconds used to calculate rotational speed
    int X = 20; //known distance between origin and right wall
    int Y = 20; //known distance between origin and front wall
    
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
            new_x = x + h * cos(theta);
            new_y = y + h * sin(theta);
            new_theta = theta;
        }
        
        //backward pulse
        else if (input == "B")
        {
            new_x = x - h * cos(theta);
            new_y = y - h * sin(theta);
            new_theta = theta;
        }
        
        //leftward pulse
        else if (input == "L")
        {
            new_x = x;
            new_y = y;
            new_theta = theta + phi;
        }
        
        //rightward pulse
        else if (input == "R")
        {
            new_x = x;
            new_y = y;
            new_theta = theta - phi;
        }
        
        //none, end simulation
        else
        {
            running = false;
            break;
        }
        
        /*      SENSOR MEASUREMENT MODELS      */
        int d_f = Y - new_y; //calculate new distance b/w robot and front wall
        int d_r = X - new_x; //calculate new distance b/w robot and right wall
        int rot_sp = (new_theta - theta) / t_interval; //calculate rotational speed
        
        //output new state and sensor measurements
        cout << "Current position: (" << new_x << ", " << new_y;
        cout << "), Current rotation angle: " << new_theta << endl;
        cout << "New front wall distance: " << d_f;
        cout << ", New right wall distance: " << d_r;
        cout << ", Rotational speed: " << rot_sp << endl;
        cout << endl;
        
        //update positions
        x = new_x;
        y = new_y;
        theta = new_theta;
    }
    
}
