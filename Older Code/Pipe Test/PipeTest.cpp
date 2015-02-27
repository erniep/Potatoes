#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <iostream>

using namespace std;

int main()
{
	
	cout << "Beginning of program." << endl;
	
	int pipe_file_desc[2]; // Pipe used for communication
	pid_t pid; //Used to determine between processes
	char message[20]; // Message used in pipes
	
		
	if (pipe(pipe_file_desc) == -1)
	{
		//Print out error message when an error creating the pipe occurs.
		cout << "Error creating pipe" << endl;
		return 0;
	}
	
	pid = fork(); // Create new process
	
	if (pid == 0)
	{
		//Child process, must read what is in the pipe
		cout << endl;
		cout << "This is the child process." << endl;
		close(pipe_file_desc[1]); // Close the write end of the pipe
		read(pipe_file_desc[0], message, 20); // Read and print the message
		cout << message << endl;
		close(pipe_file_desc[0]); // Close the read end of the file after reading the message
	}

	else if (pid > 0)
	{
		//Parent process
		cout << endl;
		cout << "This is the parent process." << endl;
		close(pipe_file_desc[0]); //Close the read end of the pipe
		write(pipe_file_desc[1], "Vegetable tempura", 20); // Write a message to the pipe
		close(pipe_file_desc[1]); // Close the write end of the file after writing a message 
	}
	
	else 
	{
		//Fork failure
		cout << "Failed attempt" << endl;
		return 1;
	}
	
	return 0;
	
}
