#include <iostream>
#include <termios.h>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <assert.h>

using namespace std;

char readKey(){
    int c;
    system("stty raw");
    c = getchar();
    system("stty cooked");
    return c;
}

int getch() {
    int c=0;
    struct termios org_opts, new_opts;
    int res=0;
        //-----  store old settings -----------
    res=tcgetattr(STDIN_FILENO, &org_opts);
    assert(res==0);
        //---- set new terminal parms --------
    memcpy(&new_opts, &org_opts, sizeof(new_opts));
    new_opts.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL | ECHOPRT | ECHOKE | ICRNL);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_opts);
    c=getchar();
        //------  restore old settings ---------
    res=tcsetattr(STDIN_FILENO, TCSANOW, &org_opts);
    assert(res==0);
    return(c);
}

char getchg() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}


int main(){
    while(true){
        int c;   

                c = getch();

                cout << c << endl;

                switch (c)
                {
                case 65:
                    cout <<  "UP_KEY" << endl;                   
                    break;
                
                case 66:
                    cout <<  "DOWN_KEY" << endl;
                    break;
                
                case 67:
                    cout <<  "RIGHT_KEY" << endl;
                    break;
                
                case 68:
                    cout << "LEFT_KEY" << endl;
                    break;
                
                default:
                    break;
                }
    }
}