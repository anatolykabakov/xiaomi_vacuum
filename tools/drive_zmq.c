/* 0.5 м вперёд и назад ЧЕРЕЗ xiaomi_robot: ZMQ PUB -> его SUB tcp://127.0.0.1:9090,
 * protobuf ZmqMessage{topic="cmd_vel", cmd_vel{vx,vy,w}}. Одометрия — player:6665 с таймаутом
 * (peek), жёсткий лимит времени, стоп при любом сбое/сигнале (защита от keep_speed). */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <zmq.h>
#include <libplayerinterface/player.h>
#include <libplayerc/playerc.h>

static void* g_pub=NULL;
/* protobuf: ZmqMessage{ 2:topic(string), 4:cmd_vel{1:vx 2:vy 3:w (double)} } */
static int enc(unsigned char*b,double vx,double vy,double w){
  int n=0; const char*t="cmd_vel";
  b[n++]=0x12; b[n++]=7; memcpy(b+n,t,7); n+=7;              /* topic */
  b[n++]=0x22; b[n++]=27;                                    /* cmd_vel, len 27 */
  double v[3]={vx,vy,w}; for(int f=0;f<3;f++){ b[n++]=((f+1)<<3)|1; memcpy(b+n,&v[f],8); n+=8; }
  return n;
}
static void send_vel(double vx){ unsigned char b[64]; int n=enc(b,vx,0,0); if(g_pub) zmq_send(g_pub,b,n,0); }
static void stop_all(void){ for(int i=0;i<5;i++){ send_vel(0.0); usleep(30000);} }
static void on_sig(int s){ (void)s; stop_all(); _exit(2); }

int main(int argc,char**argv){
  double dist=(argc>1)?atof(argv[1]):0.5, spd=(argc>2)?atof(argv[2]):0.2; int cap=((argc>3)?atoi(argv[3]):25)*10; /* лимит сегмента, с */
  int oneway=(argc>4 && strcmp(argv[4],"oneway")==0); /* только первый сегмент; знак spd = направление */
  void*ctx=zmq_ctx_new(); g_pub=zmq_socket(ctx,ZMQ_PUB);
  if(zmq_connect(g_pub,"tcp://127.0.0.1:9090")){ fprintf(stderr,"zmq connect fail\n"); return 1; }
  atexit(stop_all); signal(SIGINT,on_sig); signal(SIGTERM,on_sig); signal(SIGHUP,on_sig);
  usleep(2000000); /* slow-joiner: 2с, подписка SUB должна дойти до PUB (проверено zmq_ping) */
  for(int i=0;i<10;i++){ send_vel(0.0); usleep(100000);} /* прогрев канала нулями */
  playerc_client_t*c=playerc_client_create(NULL,"127.0.0.1",6665);
  if(!c||playerc_client_connect(c)){ fprintf(stderr,"player connect fail: %s\n",playerc_error_str()); return 1; }
  playerc_client_datamode(c,PLAYERC_DATAMODE_PULL); playerc_client_set_replace_rule(c,-1,-1,PLAYER_MSGTYPE_DATA,-1,1);
  playerc_position2d_t*p=playerc_position2d_create(c,0);
  if(!p||playerc_position2d_subscribe(p,PLAYER_OPEN_MODE)){ fprintf(stderr,"pos2d fail\n"); return 1; }
  /* чтение с таймаутом: peek 300мс, иначе считаем пропуск */
  #define READ_OK() (playerc_client_peek(c,300)>0 && playerc_client_read(c)!=NULL)
  for(int i=0;i<10 && !READ_OK();i++);
  double x0=p->px,y0=p->py; printf("канал: ZMQ->xiaomi_robot:9090 | старт x0=%.3f y0=%.3f | цель %.2f м @ %.2f м/с, лимит %d с/сегмент\n",x0,y0,dist,spd,cap/10);
  #define D() hypot(p->px-x0,p->py-y0)
  int miss=0, k;
  /* ВПЕРЁД */
  for(k=0;k<cap;k++){ send_vel(spd); if(READ_OK()) miss=0; else if(++miss>=5){ printf("!! нет одометрии — СТОП\n"); stop_all(); return 3; }
    double d=D(); if(k%5==0) printf("  вперёд: %.3f м\n",d);
    if(k==30 && d<0.005){ printf("!! колёса не реагируют (<5мм за 3с) — СТОП\n"); stop_all(); return 4; }
    if(d>=dist-0.02) break; usleep(100000); }
  stop_all(); for(int i=0;i<5;i++){ READ_OK(); usleep(100000);} printf("после вперёд: %.3f м\n",D());
  if(oneway){ printf("ФИНАЛ(oneway): смещение %.3f м (x=%.3f y=%.3f)\n",D(),p->px,p->py); return 0; }
  sleep(1);
  /* НАЗАД к старту */
  miss=0; for(k=0;k<cap;k++){ send_vel(-spd); if(READ_OK()) miss=0; else if(++miss>=5){ printf("!! нет одометрии — СТОП\n"); stop_all(); return 3; }
    double d=D(); if(k%5==0) printf("  назад: %.3f м\n",d);
    if(d<=0.04) break; usleep(100000); }
  stop_all(); for(int i=0;i<5;i++){ READ_OK(); usleep(100000);}
  printf("ФИНАЛ: до старта %.3f м (x=%.3f y=%.3f)\n",D(),p->px,p->py); return 0;
}
