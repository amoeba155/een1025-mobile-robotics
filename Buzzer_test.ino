// Call playImperialMarch() whenever you want.
// Setup once with initImperialMarch(buzzerPin, led1Pin, led2Pin).

enum{C=261,D=294,E=329,F=349,G=391,GS=415,A=440,AS=455,B=466,
     CH=523,CSH=554,DH=587,DSH=622,EH=659,FH=698,FSH=740,GH=784,GSH=830,AH=880};

static uint8_t _bz=8,_l1=12,_l2=13;
static int _cnt=0;

static inline void _beep(int n,int d){
  tone(_bz,n,d);
  uint8_t led=(_cnt++&1)?_l2:_l1;
  digitalWrite(led,HIGH); delay(d); digitalWrite(led,LOW);
  noTone(_bz); delay(50);
}

static inline void _play(const int (*s)[2], int n){
  for(int i=0;i<n;i++) (s[i][0]<0)?delay(s[i][1]):_beep(s[i][0],s[i][1]);
}

void initImperialMarch(uint8_t buzzerPin, uint8_t ledPin1, uint8_t ledPin2){
  _bz=buzzerPin; _l1=ledPin1; _l2=ledPin2;
  pinMode(_bz,OUTPUT); pinMode(_l1,OUTPUT); pinMode(_l2,OUTPUT);
}

void playImperialMarch(){
  static const int first[][2]={
    {A,500},{A,500},{A,500},{F,350},{CH,150},{A,500},{F,350},{CH,150},{A,650},{-1,500},
    {EH,500},{EH,500},{EH,500},{FH,350},{CH,150},{GS,500},{F,350},{CH,150},{A,650},{-1,500}
  };
  static const int second[][2]={
    {AH,500},{A,300},{A,150},{AH,500},{GSH,325},{GH,175},{FSH,125},{FH,125},{FSH,250},{-1,325},
    {AS,250},{DSH,500},{DH,325},{CSH,175},{CH,125},{B,125},{CH,250},{-1,350}
  };
  static const int v1[][2]={
    {F,250},{GS,500},{F,350},{A,125},{CH,500},{A,375},{CH,125},{EH,650},{-1,500}
  };
  static const int v2[][2]={
    {F,250},{GS,500},{F,375},{CH,125},{A,500},{F,375},{CH,125},{A,650},{-1,650}
  };

  _play(first,  (int)(sizeof(first)/sizeof(first[0])));
  _play(second, (int)(sizeof(second)/sizeof(second[0])));
  _play(v1,     (int)(sizeof(v1)/sizeof(v1[0])));
  _play(second, (int)(sizeof(second)/sizeof(second[0])));
  _play(v2,     (int)(sizeof(v2)/sizeof(v2[0])));
}
void setup(){
  initImperialMarch(8, 12, 13);
}
void loop(){
  playImperialMarch();
  delay(1000);
}
