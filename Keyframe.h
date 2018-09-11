class Keyframe
{
  private:
	  long position = 0;      //keyframe position [full steps]
	  unsigned long frame = 0;		  //keyframe time
	  float speed = 0;        //speed to reach keyframe position [steps/s]
	  float acc = 0;          //acceleration parameter to reach keyframe speed 
	  float dec = 0;          //deceleration to next next keyframe speed
    //Keyframe* nextKeyframe = 0; //pointer to next Keyframe in Sequence
   // Keyframe* previousKeyframe = 0;   //pointer to previous Keyframe in Sequence

  public:
    Keyframe();
	Keyframe(unsigned long frame, long position, float acc, float dec);
    Keyframe(long position, float speed);
	void setFrame(unsigned long frame);
	unsigned long getFrame();
    void setPosition(long position);
    long getPosition();
    void setSpeed(float speed);
    float getSpeed();
    void setAcc(float acc);
    float getAcc();
    void setDec(float dec);
    float getDec();
   // void setNextKeyframe(Keyframe* kf);
   // Keyframe* getNextKeyframe();
   // void setPreviousKeyframe(Keyframe* kf);
    //Keyframe* getPreviousKeyframe();
};


