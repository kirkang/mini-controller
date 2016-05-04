void OUT_30ms()
{
        if(millis()-timer1>=100)
        {

        //Serial.println(micros());
        //Serial.println(timer1);
        //Serial.println(micros()-timer1);
        
        JY901.GetAngle();
        OuT(((float)JY901.stcAngle.Angle[0]/32768*180));
        Serial.print(" ");
        OuT(((float)JY901.stcAngle.Angle[1]/32768*180));
        Serial.print(" ");
        OuT(((float)JY901.stcAngle.Angle[2]/32768*180));
        Serial.print(" ");
        
        JY901.GetAcc();
        OuT(((float)JY901.stcAcc.a[0]/32768*1600));
        Serial.print(" ");
        OuT(((float)JY901.stcAcc.a[1]/32768*1600));
        Serial.print(" ");
        OuT(((float)JY901.stcAcc.a[2]/32768*1600));
        Serial.print(" ");

        JY901.GetGyro();
        OuT((float)JY901.stcGyro.w[0]/32768*2000);
        Serial.print(" ");
        OuT((float)JY901.stcGyro.w[1]/32768*2000);
        Serial.print(" ");
        OuT((float)JY901.stcGyro.w[2]/32768*2000);
        Serial.print(" ");

        JY901.GetMag();
        OuT((float)JY901.stcMag.h[0]);
        Serial.print(" ");
        OuT((float)JY901.stcMag.h[1]);
        Serial.print(" ");
        OuT((float)JY901.stcMag.h[2]);
        Serial.print(' ');
        Serial.print('\n');
        
        timer1 = millis();
        }
}



void OuT(float n)  //输出姿态参数子函数
  {
     int m=(int)n;
     if(m>=0)
     {
        Serial.print("+");
     }
     else {
        Serial.print("-");
        m=-m;
     }
     m=m%1000;
     Serial.print(m/100);
     m=m%100;
     Serial.print(m/10);
     m=m%10;
     Serial.print(m);  
        
  }
