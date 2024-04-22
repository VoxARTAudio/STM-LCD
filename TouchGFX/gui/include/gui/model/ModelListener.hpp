#ifndef MODELLISTENER_HPP
#define MODELLISTENER_HPP

#include <gui/model/Model.hpp>

class ModelListener
{
public:
    void bind(Model* m)
    {
        model = m;
    }

protected:
    Model* model;
};

#endif /* MODELLISTENER_HPP */
