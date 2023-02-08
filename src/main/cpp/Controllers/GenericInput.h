#ifndef __COWLIB_GENERICINPUT_H__
#define __COWLIB_GENERICINPUT_H__



class GenericInput
{
    void Handle();
    virtual bool OnPress()=0;
    virtual bool OnHold()=0;
};
class Button : public GenericInput
{
public:
    Button(const int ID);
    bool OnPress();
    bool OnHold();

private:
 
};
class Trigger : public GenericInput
{
public:
    Trigger(const int ID);
    bool OnPress();
    bool OnHold();
    
private:
    
};

#endif /* __COWLIB_GENERICINPUT_H__ */