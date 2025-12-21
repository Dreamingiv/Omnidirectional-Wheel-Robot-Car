//
// Author: Breezeee
// Date: 25-5-28
//


#include "led_on_board.h"

namespace ega
{
    LEDOnBoard& LEDOnBoard::getInstance()
    {
        static LEDOnBoard instance;
        return instance;
    }

    void LEDOnBoard::init(const Config& config)
    {
        if (getInstance().inited_)
        {
            return;
        }
        getInstance().pwm_r.emplace(config.pwm_config_r);
        getInstance().pwm_g.emplace(config.pwm_config_g);
        getInstance().pwm_b.emplace(config.pwm_config_b);
        getInstance().inited_ = true;
        setColorRGB(0, 0, 0);
    }

    void LEDOnBoard::loop()
    {
        static float hue = 0.0f;

        // hue 取值范围 0~360，用于控制颜色
        hue += 2.0f;
        if (hue >= 360.0f)
            hue -= 360.0f;

        setColorHSV(hue, 1.0f, 0.3f);
    }

    void LEDOnBoard::control()
    {
        pwm_r->setDuty(static_cast<float>(rgb_.r)/255.0f);
        pwm_g->setDuty(static_cast<float>(rgb_.g)/255.0f);
        pwm_b->setDuty(static_cast<float>(rgb_.b)/255.0f);
    }

    void LEDOnBoard::HSVtoRGB()
    {
        float h = hsv_.h;
        float s = hsv_.s;
        float v = hsv_.v;
        float r{0}, g{0}, b{0};
        int i{0};
        float f{0}, p{0}, q{0}, t{0};

        if (s == 0.0f)
        {
            r = g = b = v;
        }
        else
        {
            h /= 60.0f;
            i = (int)h;
            f = h - i;
            p = v * (1.0f - s);
            q = v * (1.0f - s * f);
            t = v * (1.0f - s * (1.0f - f));

            switch (i % 6)
            {
            case 0:
                r = v;
                g = t;
                b = p;
                break;
            case 1:
                r = q;
                g = v;
                b = p;
                break;
            case 2:
                r = p;
                g = v;
                b = t;
                break;
            case 3:
                r = p;
                g = q;
                b = v;
                break;
            case 4:
                r = t;
                g = p;
                b = v;
                break;
            case 5:
                r = v;
                g = p;
                b = q;
                break;
            }
        }

        rgb_.r = (uint8_t)(r * 255);
        rgb_.g = (uint8_t)(g * 255);
        rgb_.b = (uint8_t)(b * 255);
    }

    void LEDOnBoard::setColorHSV(const float h, const float s, const float v)
    {
        getInstance().hsv_.h = h;
        getInstance().hsv_.s = s;
        getInstance().hsv_.v = v;
        getInstance().HSVtoRGB();
        getInstance().control();
    }

    void LEDOnBoard::setColorRGB(uint8_t r, uint8_t g, uint8_t b)
    {
        getInstance().rgb_.r = r;
        getInstance().rgb_.g = g;
        getInstance().rgb_.b = b;
        getInstance().control();
    }
}
