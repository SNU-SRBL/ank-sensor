/*
 * can_test.c
 *
 *  Created on: 2020. 9. 30.
 *      Author: KJI
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
#define CAN_ID 0x01
#define CAN_FRAME_MAX_LEN 8

#define p0 32793  //32767
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

int p, v, t;

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
			if(frame.data[0] == CAN_ID){
				p = (frame.data[1]<<8)|frame.data[2];
				v = (frame.data[3]<<4)|(frame.data[4]>>4);
				t = ((frame.data[4]&0xF)<<8)|frame.data[5];
				printf("%d, %d, %d\n", p, v, t);
			}
			break;
		}
		
	}

	return 0;
}
int Set_Motor_zero(const int sock)
{
	while(1){
		TransmitCanFrame(sock, CAN_ID, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID, Motor_zero, sizeof(Motor_zero));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID, Motor_off, sizeof(Motor_off));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID, Motor_on, sizeof(Motor_on));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID, Motor_rest, sizeof(Motor_rest));
		msleep(100);
		TransmitCanFrame(sock, CAN_ID, Motor_off, sizeof(Motor_off));
		msleep(100);
		ReceiveCanFrame(sock);
		msleep(100);
		printf("%d, %d\n", p, p0);
		if(p == p0){
			printf("Motor Set Zero\n");
			break;
		}
		else{
			printf("Motor zeroing Try again \n");
		}
	}
	return 0;
}

int Set_Motor_on(const int sock)
{
	TransmitCanFrame(sock, CAN_ID, Motor_on, sizeof(Motor_on));
	msleep(100);
	return 0;
}

int Set_Motor_off(const int sock)
{
	TransmitCanFrame(sock, CAN_ID, Motor_off, sizeof(Motor_off));
	msleep(100);
	return 0;
}

int main(void)
{
	int sock = InitCanInterface("can0");
	if (sock < 0) {
		return -1;
	}
	int p_input = p0;
	int v_input = v0;
	int kp_input = 1000;
	int kv_input = 200;
	double t_input = 2047; // 1747~2247
	
	Set_Motor_zero(sock);
	Set_Motor_on(sock);
	//Set_Motor_off(sock);
	
	uint8_t Motor_control[CAN_FRAME_MAX_LEN];
	

	uint8_t can_data[CAN_FRAME_MAX_LEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}; 
	//TransmitCanFrame(sock, CAN_ID, can_data, sizeof(can_data));
	//ReceiveCanFrame(sock);
	//msleep(100);
	
	for (int i=0; i<270; i++){
		p_input = p_input-7;

		Motor_control[0] = p_input>>8;
		Motor_control[1] = p_input&0xFF;
		Motor_control[2] = v_input>>4;
		Motor_control[3] = ((v_input&0xF)<<4)+(kp_input>>8);
		Motor_control[4] = kp_input&0xFF;
		Motor_control[5] = kv_input>>4;
		Motor_control[6] = ((kv_input&0xF)<<4)+((int)t_input>>8);
		Motor_control[7] = (int)t_input&0xFF;
	 
		TransmitCanFrame(sock, CAN_ID, Motor_control, sizeof(Motor_control));
		ReceiveCanFrame(sock);
		msleep(30);
	}
	
	for (int j=0; j<10; j++){
		for (int i=0; i<540; i++){
			p_input = p_input+7;

			Motor_control[0] = p_input>>8;
			Motor_control[1] = p_input&0xFF;
			Motor_control[2] = v_input>>4;
			Motor_control[3] = ((v_input&0xF)<<4)+(kp_input>>8);
			Motor_control[4] = kp_input&0xFF;
			Motor_control[5] = kv_input>>4;
			Motor_control[6] = ((kv_input&0xF)<<4)+((int)t_input>>8);
			Motor_control[7] = (int)t_input&0xFF;
		 
			TransmitCanFrame(sock, CAN_ID, Motor_control, sizeof(Motor_control));
			ReceiveCanFrame(sock);
			msleep(30);
		}
		
		for (int i=0; i<540; i++){
			p_input = p_input-7;

			Motor_control[0] = p_input>>8;
			Motor_control[1] = p_input&0xFF;
			Motor_control[2] = v_input>>4;
			Motor_control[3] = ((v_input&0xF)<<4)+(kp_input>>8);
			Motor_control[4] = kp_input&0xFF;
			Motor_control[5] = kv_input>>4;
			Motor_control[6] = ((kv_input&0xF)<<4)+((int)t_input>>8);
			Motor_control[7] = (int)t_input&0xFF;
		 
			TransmitCanFrame(sock, CAN_ID, Motor_control, sizeof(Motor_control));
			ReceiveCanFrame(sock);
			msleep(30);
		}
	}
	
	for (int i=0; i<270; i++){
		p_input = p_input+7;

		Motor_control[0] = p_input>>8;
		Motor_control[1] = p_input&0xFF;
		Motor_control[2] = v_input>>4;
		Motor_control[3] = ((v_input&0xF)<<4)+(kp_input>>8);
		Motor_control[4] = kp_input&0xFF;
		Motor_control[5] = kv_input>>4;
		Motor_control[6] = ((kv_input&0xF)<<4)+((int)t_input>>8);
		Motor_control[7] = (int)t_input&0xFF;
	 
		TransmitCanFrame(sock, CAN_ID, Motor_control, sizeof(Motor_control));
		ReceiveCanFrame(sock);
		msleep(30);
	}
	
	return 0;
}
