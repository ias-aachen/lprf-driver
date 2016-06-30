#include <stdio.h>
#include <stdlib.h>

int main()
{
	FILE * fp;
	int c;
	int i = 0;
	long pos;
	fp = fopen ("/dev/lprf", "r+");

	printf ("Enter requested address (hex), 0 to cancel: ");
	scanf ("%X",&i);  
//	while(i != 0)
//	{
		printf("fseek...\n");
		fseek(fp, i, SEEK_SET);  //calls kernel file_operations.llseek but also to file_operations.read
		printf("fseek finished. ftell...\n");
		pos = ftell(fp);
		printf("ftell finished. fgetc...\n");		
		c = fgetc(fp);
		printf("fgetc finished\n");
		printf("pos=%ld \t c=%02X \t c=%d \n", pos, c, c);		

//		printf ("Enter requested address (hex), 0 to cancel: ");
//		scanf ("%X",&i);  		
//	}

	fclose(fp);
				   
	return(0);
}
