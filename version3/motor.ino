#define EA 3
#define INA1 2
#define INA2 4

#define EB 6
#define INB1 5
#define INB2 7

#define EC 9
#define INC1 8
#define INC2 10

#define ED 11
#define IND1 12
#define IND2 13

//int N[34];

MotorControl Amotor(EA,INA1,INA2);
MotorControl Bmotor(EB,INB1,INB2);
MotorControl Cmotor(EC,INC1,INC2);
MotorControl Dmotor(ED,IND1,IND2);



//int Motor_Read(){
//  if(Serial.available())
//  {
//    
//    if(Serial.available()==18)            //满足要求29个字
//    {
//      //Serial.println(Serial.available());
//      for(int i=1;i<=18;i++)
//      {         
//        N[i]=Serial.read();    
//      }
//      //Serial.println("ok");
//      //Serial.println(Serial.available());
//      //while(Serial.available() != 0){Serial.read();}
//      return 1;
//    }
//    else{
//    
//    if(Serial.available()>18){
//       while(Serial.available()){Serial.read();}
//       }       //不满足要求，清楚缓存
//    }
//  }
//  return 0;
//}
//
//
//
//void action()
//{
//  if(N[17]==(int)'\r'&&N[18]==(int)'\n')           //双重确定
//  {
//     N[17]=0;
//     N[18]=0;
//     if(N[1]=='+'){
//       Amotor.Move(1, (N[2]-48)*100+(N[3]-48)*10+(N[4]-48));
//     }
//     else if(N[1]=='-'){
//       Amotor.Move(0, (N[2]-48)*100+(N[3]-48)*10+(N[4]-48));
//     }
//     
//     if(N[5]=='+'){
//       Bmotor.Move(1, (N[6]-48)*100+(N[7]-48)*10+(N[8]-48));
//     }
//     else if(N[5]=='-'){
//       Bmotor.Move(0, (N[6]-48)*100+(N[7]-48)*10+(N[8]-48));
//     }
//     
//     
//     if(N[9]=='+'){
//       Cmotor.Move(1, (N[10]-48)*100+(N[11]-48)*10+(N[12]-48));
//     }
//     else if(N[9]=='-'){
//       Cmotor.Move(0, (N[10]-48)*100+(N[11]-48)*10+(N[12]-48));
//     }
//     
//     if(N[13]=='+'){
//       Dmotor.Move(1, (N[14]-48)*100+(N[15]-48)*10+(N[16]-48));
//     }
//     else if(N[13]=='-'){
//       Dmotor.Move(0, (N[14]-48)*100+(N[15]-48)*10+(N[16]-48));
//     }
//     
//    
//  }
//}

