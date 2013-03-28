/* 
 * File:   kbhit.h
 * Author: jarvis
 *
 * Created on December 14, 2010, 4:23 PM
 */

/* kbhit.h */
#include <sys/select.h>
#include <sys/types.h>

int kbhit(void)
{
    /* struct timeval tv; */
    /* fd_set read_fd; */
    /* tv.tv_sec=0; */
    /* tv.tv_usec=0; */
    /* FD_ZERO(&read_fd); */
    /* FD_SET(0,&read_fd); */
    
    /* if(select(1, &read_fd, NULL, NULL, &tv) == -1) return 0; */
    
    /* if(FD_ISSET(0,&read_fd)) return 1; */

    /* return 0; */

    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if(ch != EOF)
    {
	ungetc(ch, stdin);
	return 1;
    }

    return 0;
}
