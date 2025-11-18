/*
 * can_test.c
 */

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <errno.h>
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

#define CAN_ID_Master 0x00
#define CAN_ID_Motor_1 0x01
#define CAN_ID_Motor_2 0x02
#define CAN_FRAME_MAX_LEN 8

#define p0 32767    //32767
#define p0_2 32618 //32819
#define v0 2047
#define kp0 0
#define kv0 0
#define t0 2047

#define p_min 29422
#define p_max 32767

uint8_t Motor_on[CAN_FRAME_MAX_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; // Motor on
uint8_t Motor_off[CAN_FRAME_MAX_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC}; // Motor off
uint8_t Motor_zero[CAN_FRAME_MAX_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; // Motor zero
uint8_t Motor_rest[CAN_FRAME_MAX_LEN] = {p0>>8, p0&0xFF, v0>>4, ((v0&0xF)<<4)+(kp0>>8), kp0&0xFF, kv0>>4, ((kv0&0xFF)<<4)+(t0>>8), t0&0xFF}; // Motor rest

int p1, v1, t1;
int p2, v2, t2;

int msleep(long msec)
{
	struct timespec ts;
	int res;
	
	if(msec < 0)
	{
		errno = EINVAL;
		return -1;		
	}	
	
	ts.tv_sec = msec/1000;
	ts.tv_nsec = (msec%1000)*1000000;
	
	do{
		res = nanosleep(&ts, &ts);
	} while (res && errno == EINTR);
	
	return res;
}	
	
int InitCanInterface(const char *ifname)
{
	int sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (sock == -1) {
		printf("Fail to create can socket for %s - %m\n", ifname);
		return -1;
	}
	printf("Success to create can socket for %s\n", ifname);

	struct ifreq ifr;
	strcpy(ifr.ifr_name, ifname);
	int ret = ioctl(sock, SIOCGIFINDEX, &ifr);
	if (ret == -1) {
		perror("Fail to get can interface index -");
		return -1;
	}
	printf("Success to get can interface index: %d\n", ifr.ifr_ifindex);

	struct sockaddr_can addr;
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	ret = bind(sock, (struct sockaddr *)&addr, sizeof(addr));
	if (ret == -1) {
		perror("Fail to bind can socket -");
		return -1;
	}
	printf("Success to bind can socket\n");

	return sock;
}


int TransmitCanFrame(const int sock, const uint32_t id, const uint8_t *data, const size_t data_len)
{
	struct can_frame frame;
	frame.can_id = id & 0x1fffffff;
	//frame.can_id |= (1 << 31);
	memcpy(frame.data, data, data_len);
	frame.can_dlc = data_len;
	
	int tx_bytes = write(sock, &frame, sizeof(struct can_frame));
	if (tx_bytes == -1) {
		perror("Fail to transmit can frame -");
		return -1;
	}
	printf("Success to transmit can frame - %d bytes is transmitted\n", tx_bytes);
	return 0;
}

int ReceiveCanFrame(const int sock)
{
	struct can_frame frame;
	while(1){
		int rx_bytes = read(sock, &frame, sizeof(struct can_frame));
		//printf("rx_btyes %d, id: %X, dlc: %X, \n",rx_bytes,frame.can_id,frame.can_dlc);
		printf("rx_btyes %d, id: %d, dlc: %d, \n",rx_bytes,frame.can_id,frame.can_dlc);
		printf("%X\n",frame.data[0]);
		if (rx_bytes < 0) {
			perror("Fail to receive can frame -");
			return -1;
		} else if (rx_bytes < (int)sizeof(struct can_frame)) {
			printf("Incomplete can frame is received - rx_bytes: %d\n", rx_bytes);
			return -1;
		} else if (frame.can_dlc > CAN_FRAME_MAX_LEN) {
			printf("Invalid dlc: %d\n", frame.can_dlc);
			return -1;
		}
		
		if (frame.can_id == CAN_ID_Master){
			if(frame.data[0] == CAN_ID_Motor_1){
				p1 = (frame.data[1]<<8)|frame.data[2];
				v1 = (frame.data[3]<<4)|(frame.data[4]>>4);
				t1 = ((frame.data[4]&0xF)<<8)|frame.data[5];
				printf("M1, %d, %d, %d\n", p1, v1, t1);
			}

			if(frame.data[0] == CAN_ID_Motor_2){
				p2 = (frame.data[1]<<8)|frame.data[2];
				v2 = (frame.data[3]<<4)|(frame.data[4]>>4);
				t2 = ((frame.data[4]&0xF)<<8)|frame.data[5];
				printf("M2, %d, %d, %d\n", p2, v2, t2);
			}
			break;			
		}
	}

	return 0;
}
int Set_Motor_zero(const int sock)
{
	
	while(1){
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_zero, sizeof(Motor_zero));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_on, sizeof(Motor_on));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_rest, sizeof(Motor_rest));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_off, sizeof(Motor_off));
		msleep(100);
		ReceiveCanFrame(sock);
		msleep(100);
		printf("M1, %d, %d\n", p1, p0);
		if(p1 == p0){
			printf("Motor_1 Set Zero\n");
			break;
		}
		else{
			printf("Motor_1 zeroing Try again \n");
		}
	}

	while(1){
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_zero, sizeof(Motor_zero));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_on, sizeof(Motor_on));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_rest, sizeof(Motor_rest));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_off, sizeof(Motor_off));
		msleep(100);
		ReceiveCanFrame(sock);
		msleep(100);
		printf("M2, %d, %d\n", p2, p0_2);
		if(p2 == p0_2){
			printf("Motor_2 Set Zero\n");
			break;
		}
		else{
			printf("Motor_2 zeroing Try again \n");
		}
	}
	
	return 0;
}

int Set_Motor_on(const int sock)
{
	TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_on, sizeof(Motor_on));
	msleep(100);
	TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_on, sizeof(Motor_on));
	msleep(100);
	return 0;
}

int Set_Motor_off(const int sock)
{
	TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_off, sizeof(Motor_off));
	msleep(100);
	TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_off, sizeof(Motor_off));
	msleep(100);
	return 0;
}

int main(void)
{
	int sock = InitCanInterface("can0");
	if (sock < 0) {
		return -1;
	}
	int p1_input = p0;
	int v1_input = v0;
	int kp1_input = 1000;
	int kv1_input = 200;
	double t1_input = 2247; // 1747~2247
	int p2_input = p0_2;
	int v2_input = v0;
	int kp2_input = 1000;
	int kv2_input = 200;
	double t2_input = 2247; // 1747~2247

	Set_Motor_zero(sock);
	Set_Motor_on(sock);
	//Set_Motor_off(sock);
	
	uint8_t Motor_control_1[CAN_FRAME_MAX_LEN];
	uint8_t Motor_control_2[CAN_FRAME_MAX_LEN];
	

	//uint8_t can_data[CAN_FRAME_MAX_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; 
	//TransmitCanFrame(sock, CAN_ID, can_data, sizeof(can_data));
	//ReceiveCanFrame(sock);
	//msleep(100);
	for (int i=0; i<450; i++){
		p1_input = p1_input-5;
		p2_input = p2_input+5;

		Motor_control_1[0] = p1_input>>8;
		Motor_control_1[1] = p1_input&0xFF;
		Motor_control_1[2] = v1_input>>4;
		Motor_control_1[3] = ((v1_input&0xF)<<4)+(kp1_input>>8);
		Motor_control_1[4] = kp1_input&0xFF;
		Motor_control_1[5] = kv1_input>>4;
		Motor_control_1[6] = ((kv1_input&0xF)<<4)+((int)t1_input>>8);
		Motor_control_1[7] = (int)t1_input&0xFF;

		Motor_control_2[0] = p2_input>>8;
		Motor_control_2[1] = p2_input&0xFF;
		Motor_control_2[2] = v2_input>>4;
		Motor_control_2[3] = ((v2_input&0xF)<<4)+(kp2_input>>8);
		Motor_control_2[4] = kp2_input&0xFF;
		Motor_control_2[5] = kv2_input>>4;
		Motor_control_2[6] = ((kv2_input&0xF)<<4)+((int)t2_input>>8);
		Motor_control_2[7] = (int)t2_input&0xFF;
	 
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_control_1, sizeof(Motor_control_1));
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_control_2, sizeof(Motor_control_2));
		ReceiveCanFrame(sock);
		msleep(10);
	}
	for (int j=0; j<10; j++){		
		for (int i=0; i<900; i++){
			p1_input = p1_input+5;
			p2_input = p2_input-5;

			Motor_control_1[0] = p1_input>>8;
			Motor_control_1[1] = p1_input&0xFF;
			Motor_control_1[2] = v1_input>>4;
			Motor_control_1[3] = ((v1_input&0xF)<<4)+(kp1_input>>8);
			Motor_control_1[4] = kp1_input&0xFF;
			Motor_control_1[5] = kv1_input>>4;
			Motor_control_1[6] = ((kv1_input&0xF)<<4)+((int)t1_input>>8);
			Motor_control_1[7] = (int)t1_input&0xFF;

			Motor_control_2[0] = p2_input>>8;
			Motor_control_2[1] = p2_input&0xFF;
			Motor_control_2[2] = v2_input>>4;
			Motor_control_2[3] = ((v2_input&0xF)<<4)+(kp2_input>>8);
			Motor_control_2[4] = kp2_input&0xFF;
			Motor_control_2[5] = kv2_input>>4;
			Motor_control_2[6] = ((kv2_input&0xF)<<4)+((int)t2_input>>8);
			Motor_control_2[7] = (int)t2_input&0xFF;
		 
			TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_control_1, sizeof(Motor_control_1));
			TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_control_2, sizeof(Motor_control_2));
			ReceiveCanFrame(sock);
			msleep(10);
		}
		
		for (int i=0; i<900; i++){
			p1_input = p1_input-5;
			p2_input = p2_input+5;

			Motor_control_1[0] = p1_input>>8;
			Motor_control_1[1] = p1_input&0xFF;
			Motor_control_1[2] = v1_input>>4;
			Motor_control_1[3] = ((v1_input&0xF)<<4)+(kp1_input>>8);
			Motor_control_1[4] = kp1_input&0xFF;
			Motor_control_1[5] = kv1_input>>4;
			Motor_control_1[6] = ((kv1_input&0xF)<<4)+((int)t1_input>>8);
			Motor_control_1[7] = (int)t1_input&0xFF;

			Motor_control_2[0] = p2_input>>8;
			Motor_control_2[1] = p2_input&0xFF;
			Motor_control_2[2] = v2_input>>4;
			Motor_control_2[3] = ((v2_input&0xF)<<4)+(kp2_input>>8);
			Motor_control_2[4] = kp2_input&0xFF;
			Motor_control_2[5] = kv2_input>>4;
			Motor_control_2[6] = ((kv2_input&0xF)<<4)+((int)t2_input>>8);
			Motor_control_2[7] = (int)t2_input&0xFF;
		 
			TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_control_1, sizeof(Motor_control_1));
			TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_control_2, sizeof(Motor_control_2));
			ReceiveCanFrame(sock);
			msleep(10);
		}
		printf("%d cycle \n",j);

	}
	for (int i=0; i<450; i++){
		p1_input = p1_input+5;
		p2_input = p2_input-5;

		Motor_control_1[0] = p1_input>>8;
		Motor_control_1[1] = p1_input&0xFF;
		Motor_control_1[2] = v1_input>>4;
		Motor_control_1[3] = ((v1_input&0xF)<<4)+(kp1_input>>8);
		Motor_control_1[4] = kp1_input&0xFF;
		Motor_control_1[5] = kv1_input>>4;
		Motor_control_1[6] = ((kv1_input&0xF)<<4)+((int)t1_input>>8);
		Motor_control_1[7] = (int)t1_input&0xFF;

		Motor_control_2[0] = p2_input>>8;
		Motor_control_2[1] = p2_input&0xFF;
		Motor_control_2[2] = v2_input>>4;
		Motor_control_2[3] = ((v2_input&0xF)<<4)+(kp2_input>>8);
		Motor_control_2[4] = kp2_input&0xFF;
		Motor_control_2[5] = kv2_input>>4;
		Motor_control_2[6] = ((kv2_input&0xF)<<4)+((int)t2_input>>8);
		Motor_control_2[7] = (int)t2_input&0xFF;
	 
		TransmitCanFrame(sock, CAN_ID_Motor_1, Motor_control_1, sizeof(Motor_control_1));
		TransmitCanFrame(sock, CAN_ID_Motor_2, Motor_control_2, sizeof(Motor_control_2));
		ReceiveCanFrame(sock);
		msleep(10);
	}

	return 0;
}
