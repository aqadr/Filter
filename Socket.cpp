// Implementation of the Socket class.


#include "Socket.h"
#include "string.h"
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <iostream>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <sys/wait.h>
#include <cstdlib>



using namespace std;


// get sockaddr, IPv4 or IPv6
void *get_in_addr(struct sockaddr *sa) {
  
  if (sa->sa_family ==AF_INET) {
    return &(((struct sockaddr_in*)sa)->sin_addr);
  }
  return &(((struct sockaddr_in6*)sa)->sin6_addr);
}



Socket::Socket() : sockfd ( -1 ), yes(1), buf_size(5500) {

    memset ( &hints, 0, sizeof ( hints ) );
    //yes =1;
}


Socket::~Socket() {
    if ( is_valid() )::close ( sockfd );
}

void sigchld_handler (int s) {
  
  while(waitpid(-1, NULL, WNOHANG) >0);
}







int Socket::create_server() {
  
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = AI_PASSIVE;   //use my IP
  
  
  
  if ((rv = getaddrinfo(NULL, MYPORT, &hints, &servinfo)) !=0) {
    
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }
  
  //loop through all the results and bind to the first we can
  for (p =servinfo; p != NULL; p = p->ai_next) {
    
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      
      perror ("Server:socket");
      continue;
    }
    
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(int)) == -1) {
      
      perror("setsockopt");
      exit(1);
    }
    
    if (bind(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      
      close(sockfd);
      perror("server:bind");
      continue;
    }
    
    break;
  }
  
  if (p == NULL) {
    
    fprintf(stderr, "server: failed to bind\n");
    return 2;
  }
  
  inet_ntop(p->ai_family, get_in_addr((struct sockaddr *)p->ai_addr), s,sizeof(s));
  printf("client: connecting to %s\n", s);
  
  freeaddrinfo(servinfo);	//all done with this structure
  
    
}
  

int Socket::create_client() {
  
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  

  // get the server address info
  if ((rv = getaddrinfo("172.16.233.136", MYPORT, &hints, &servinfo)) !=0) {
    
    fprintf(stderr, "getaddrinfo: %s\n", gai_strerror(rv));
    return 1;
  }
  
  //loop through all the results and bind to the first we can
  for (p =servinfo; p != NULL; p = p->ai_next) {
    
    if ((sockfd = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
      
      perror ("client:socket");
      continue;
    }
    
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buf_size, sizeof(int)) == -1) {
      
      perror("setsockopt");
      exit(1);
    }
   
    if (connect(sockfd, p->ai_addr, p->ai_addrlen) == -1) {
      close(sockfd);
      perror("client:connect");
      continue;
    }
    
    
    
  //  set_non_blocking ( true );
    break;
  }
  
  if (p == NULL) {
    
    fprintf(stderr, "client: failed to connect\n");
    return 2;
  }
 
  freeaddrinfo(servinfo);	//all done with this structure
  
}  
  
bool Socket::listen() {
  if ( ! is_valid() ) {
      return false;
  }
  int listen_return = ::listen ( sockfd, MAXCONNECTIONS );
  
  if ( listen_return == -1 ) {
    perror("listen");  
    return false;
  }

  return true;
}

bool Socket::reapalldead() {
  
  sa.sa_handler = sigchld_handler;
  sigemptyset (&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  if (sigaction(SIGCHLD, &sa, NULL) == -1) {
    perror("sigaction");
    return false;
  }

  return true;
}



bool Socket::accept ( )
{
  
  socklen_t addr_length = sizeof ( their_addr );
  new_fd = ::accept ( sockfd, ( struct sockaddr * )&their_addr,&addr_length );
  cout<<"new_fd:"<<new_fd<<endl;
  if ( new_fd == -1 ) {
    perror ("accept");
    return false;
  } else {
    
    return true;
  }
} 
 

 void Socket::client_info() {
   
  inet_ntop(their_addr.ss_family, get_in_addr((struct sockaddr *)&their_addr),s,sizeof(s));
  printf("server: got connection from %s\n", s);
}
  
 

  

  










int Socket::send_data (char *p, size_t size)

 {
  
  int status = send(sockfd, p, size , 0);//int flags);
  if ( status == -1 )
    {
      std::cout<<"can not send"<<std::endl;
      return -1;
    }
  else
    {
      std::cout<<"status"<<status<<std::endl;
      return status;
    }
}










/*int opts;

  opts = fcntl ( m_sock,
		 F_GETFL );

  opts = ( opts | O_NONBLOCK );
  
  fcntl ( m_sock,
	  F_SETFL,opts );


freeaddrinfo(servinfo);
}


int Socket::send ( std::string& s)

{
  socklen_t addr_len = sizeof(their_addr);
  int status = ::sendto(sockfd, s.c_str(), s.size(), 0, (struct sockaddr *)&their_addr, addr_len);
 
  if ( status == -1 )
    {
      std::cout << "status == -1   errno == " << errno << "  in Socket::recvfrom\n";
      return 0;
    }
  else if ( status == 0 )
    {
      
      cout<<"try"<<endl;
      return 0;
    }
   else 
    {
      return status;
    }
}

int Socket::sendpatch (unsigned char *p, size_t size)

 {
    cout<<"check value:"<<p[4]<<endl;
    cout<<"socketfd:"<<sockfd<<endl;
    cout<<"size of data:"<<size<<endl;
    socklen_t addr_len =sizeof(their_addr);
    cout<<"addr_len:"<<addr_len<<endl;
    cout<<"m_addr len:"<<sizeof(m_addr)<<endl;
  int status = ::sendto ( sockfd, p, size, 0, ( struct sockaddr *)&their_addr, addr_len );
  //int status = ::send(sockfd, p, size , 0);//int flags);
  if ( status == -1 )
    {
      std::cout<<"can not send"<<std::endl;
      return -1;
    }
  else
    {
      std::cout<<"status"<<status<<std::endl;
      return status;
    }


 }


 int Socket::sendsensor ( long *data, size_t size)

 {
   
    socklen_t addr_len =sizeof(their_addr);
  int status = ::sendto ( sockfd, (char *)data, size, 0, ( struct sockaddr *)&their_addr, addr_len );
  if ( status == -1 )
    {
      std::cout<<"can not send"<<std::endl;
      return -1;
    }
  else
    {
      std::cout<<"status"<<status<<std::endl;
      return status;
    }


 }
 
 
 
int Socket::recv (std::string& s)
{
    char *ptr;
    ptr=buf;

    num_of_bytes_available =0;  
    if(ioctl(sockfd, FIONREAD, & num_of_bytes_available) <0) {
	    cout<<"no data"<<endl;
	}   else {
	    cout<<"no_of_bytes=="<<num_of_bytes_available<<endl;
	}
    memset ( ptr, 0, 100);
    int status = ::recvfrom( sockfd, ptr, 99, 0,( struct sockaddr *)&their_addr, &addr_len );
    
    if ( status == -1 ) {
	     std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
	     return 0;
	 } 
    else if ( status == 0 ) {
	    return 0;
	 }
    else {
	    s = buf;
	    return status;
	 }
}

*/

int Socket::recv_data(char *ptr ) {
 
    num_of_bytes_available =0;  
    
    if(ioctl(sockfd, FIONREAD, & num_of_bytes_available) <0) { 
//	std::cout<<"no data"<<endl;
    }else {
	memset ( ptr, 0, 5500);
	int status = recv( sockfd, ptr, 5500, 0 );
        if ( status == -1 ) {
	     std::cout << "status == -1   errno == " << errno << "  in Socket::recv\n";
	     return 1;
	} else {
	    return 0;
	} 
    }
}









/*

bool Socket::connect ( const std::string host, const int port )
{
  if ( ! is_valid() ) return false;

  m_addr.sin_family = AF_INET;
  m_addr.sin_port = htons ( port );

  int status = inet_pton ( AF_INET, host.c_str(), &m_addr.sin_addr );

  if ( errno == EAFNOSUPPORT ) return false;

  status = ::connect ( m_sock, ( sockaddr * ) &m_addr, sizeof ( m_addr ) );

  if ( status == 0 )
    return true;
  else
    return false;
}

*/

void Socket::set_non_blocking ( const bool b ) {

  int opts;

  opts = fcntl ( sockfd,F_GETFL );

  if ( opts < 0 ) {
      return;
  }

  if ( b )
    opts = ( opts | O_NONBLOCK );
  else
    opts = ( opts & ~O_NONBLOCK );

  fcntl ( sockfd, F_SETFL,opts );

}

