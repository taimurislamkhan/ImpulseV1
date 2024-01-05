#ifndef MODEL_HPP
#define MODEL_HPP

class ModelListener;

class Model
{
public:
    Model();

    void bind(ModelListener* listener)
    {
        modelListener = listener;
    }

    void tick();
protected:
    ModelListener* modelListener;
};

#endif // MODEL_HPP


//#ifndef MODEL_HPP
//#define MODEL_HPP
//
//class ModelListener;
//
//class Model
//{
//public:
//    Model();
//
//    void bind(ModelListener* listener)
//    {
//        modelListener = listener;
//    }
//
//    void tick();
//
//    void screenWasTouched()
//    {
//        screenTouched = true;
//    }
//
//    bool wasScreenTouched() const
//    {
//        return screenTouched;
//    }
//
//    void resetScreenTouchedFlag()
//    {
//        screenTouched = false;
//    }
//
//protected:
//    ModelListener* modelListener;
//    bool screenTouched = false;  // flag indicating if the screen was touched
//};
//
//#endif // MODEL_HPP
