#include "Encoder.h"
#include "Config.h"

/**
 * TODO -- 
 *      > If using moving filter, use a N-frame value difference to compute revolutions
 * 
 */

static constexpr float Encoder_AngleFactor = (360.0f / 4096.0f);
// Degrees required to trigger rotation
static constexpr float Encoder_AngleDiffThresh = 350.0f;

//==============================================================================
EncoderMonitor::EncoderMonitor() : m_MovAvg(m_FltSize) {}

//==============================================================================
EncoderMonitor::~EncoderMonitor() {}

//==============================================================================
void EncoderMonitor::update(float angle) {
    // If we're using a filter, process it here. This is optimized away if m_FltSize is set to <= 1
    if (m_FltSize > 1) {
        m_MovAvg.push(angle);
        angle = m_MovAvg.get();
    }

    // Check if a rev has happened
    if (m_Initialized){
        const float dif = angle - m_LastAngle;

        m_Dir = (dif < 0.0f) ? -1.0f : (dif > 0.0f) ? 1.0f : 0.0f;

        /*
            If diff goes from 359 -> 0.0 (diff == -359.0), that means we rotated once positively.
            If diff goes from 0.0 -> 359 (diff == 359.0), that means we rotated once negatively. 
        */

        // Revolution occured
        if (fabsf(dif) > Encoder_AngleDiffThresh){
            // Positive revolution
            if (m_Dir < 0.0)
                ++m_Revs;
            else
                --m_Revs;
        }
   }
    else m_Initialized = true;
    m_TotalAngle = ((float)m_Revs * 360.0f) + angle;
    m_LastAngle = angle;
}

//==============================================================================
int32_t EncoderMonitor::getTotalRevs() const {
    return m_Revs;
}

//==============================================================================
float EncoderMonitor::getTotal() const {
    return m_TotalAngle;
}
