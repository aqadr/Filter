

#include "ClientSocket.h"
#include "SocketException.h"


ClientSocket::ClientSocket (char *host, int port)
{
  if ( ! Socket::create(host) )
    {
      throw SocketException ( "Could not create client socket." );
    }
     ptr=Socket::ptr;

}


int ClientSocket::recv_data (std::string& s)

{
	  int status = Socket::recv(s);
	  if (status == -1)
	      {
      
		  throw SocketException ( "Could not read from socket." );
	      }
	  else 
	      {
		  return status;
	      }
 
 
} 

int  ClientSocket::recvpatch (char *p)
{
   
    int status = Socket::recv_patch(p);
	  if (status == -1)
	      {
      
		  throw SocketException ( "Could not read from socket." );
	      }
	  else 
	      {
		  return status;
	      }
 
 }  


 
