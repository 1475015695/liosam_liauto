#include<iostream>
#include<cstring>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<netinet/in.h>
#include<arpa/inet.h>
using namespace std;
int main()
{
	int client_sockfd;
	int len;
	struct sockaddr_in remote_addr;
   	char recv_buf[BUFSIZ];//数据接收缓冲区
   	char send_buf[BUFSIZ];//数据传输缓冲区
	memset(&remote_addr, 0, sizeof(remote_addr));
	remote_addr.sin_family=AF_INET;//设置为IP通信
	remote_addr.sin_addr.s_addr=inet_addr("127.0.0.1");//服务器IP地址
	remote_addr.sin_port=htons(8000);//服务器端口号
	
	//创建客户端套接字 IPv4 tcp
	if((client_sockfd=socket(PF_INET, SOCK_STREAM, 0))<0)
	{
		cout<<"socket error";
		return 1;
	}
	
	//绑定服务器网络地址
	if(connect(client_sockfd, (struct sockaddr*)&remote_addr, sizeof(struct sockaddr))<0)
	{
		cout<<"connect error";
		return 1;
	} 
	
	cout<<"connected to server"<<endl;;
	len=recv(client_sockfd, recv_buf, BUFSIZ, 0);//接受服务端消息
	recv_buf[len] = '\0';
	
	//循环的发送接受信息并打印接受信息（可以按需发送）--struct sockaddr))<0
	while(1)
	{
		//发送消息给服务端
		cout<<"Enter string to send:";
		cin>>send_buf;
		if(!strcmp(send_buf, "quit")) break;
		len=send(client_sockfd, send_buf, strlen(send_buf), 0);
		
		//从客户端接受消息
		len=recv(client_sockfd,recv_buf,BUFSIZ,0);
        if(len > 0){
		recv_buf[len]='\0';
		cout<<"Received："<<recv_buf<<" ，Info Length："<<len<<endl;        	
        }
	}
	
	close(client_sockfd);
	return 0;
}
 
 