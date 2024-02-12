/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "em_cmu.h"
#include "em_emu.h"

#define HFRCOEM23_DESIRED_FREQ      20000000

#if (HFRCOEM23_DESIRED_FREQ == 20000000)
#define HFRCOEM23_CAL_INDEX         9
#else
#define HFRCOEM23_CAL_INDEX         8//19MHz otherwise
#endif

#define DOWNCOUNT                   0xFFFFF
#define PERFORM_FINETUNE            1


typedef enum HFRCO_Calibration_State {
  HFRCO_CALIBRATION_OFF = 0,
  HFRCO_CALIBRATION_IDLE,
  HFRCO_CALIBRATION_TUNING,
  HFRCO_CALIBRATION_FINE_TUNING,
  HFRCO_CALIBRATION_DONE,
}HFRCO_Calibration_State_t;

bool *tuningOver;

// Global variables used in calibration ISR // TODO - could be an object like typedef struct
static HFRCO_Calibration_State_t calibrationState;
static bool tuned, finetuned, lastCalcUpcountGT, lastCalcUpcountLT;
static uint32_t theoreticalUpcount, previousUpcount, currentTuningVal, newTuningVal;

static void resetCmuCalibrationFsm(void);

//Emlib Extensions
uint32_t CMU_OscillatorFineTuningGet(CMU_Osc_TypeDef osc);
void CMU_OscillatorFineTuningSet(CMU_Osc_TypeDef osc, uint32_t val);

static void initHFRCOEM23(void){
  CMU_ClockEnable(cmuClock_HFRCOEM23, true);

  HFRCOEM23->CAL = DEVINFO->HFRCOEM23CAL[HFRCOEM23_CAL_INDEX].HFRCOEM23CAL;//Calibration indexes are available in RM section 9.3.3.3
  //HFRCOEM23->CTRL |= HFRCO_CTRL_EM23ONDEMAND;
  HFRCOEM23->CTRL |= HFRCO_CTRL_FORCEEN;//We keep HFRCO enabled for debug purposes since no other peripheral consumes it
  // Drive HFRCO onto PB5 to observe calibration
  CMU_ClkOutPinConfig(2, cmuSelect_HFRCOEM23, 4, gpioPortB, 5);
}

static void initCmuCalibrationFsm(void)
{
  resetCmuCalibrationFsm();

  // Determine ideal up count based on the desired frequency
  theoreticalUpcount = (uint32_t)(((float)HFRCOEM23_DESIRED_FREQ / (float)SystemHFXOClockGet()) * (float)(DOWNCOUNT + 1));

  // Setup the calibration circuit
  CMU_CalibrateConfig(DOWNCOUNT, cmuSelect_HFXO, cmuSelect_HFRCOEM23);

  // Enable continuous calibration
  CMU_CalibrateCont(true);

  // Enable calibration ready interrupt
  CMU_IntClear(_CMU_IF_MASK);
  CMU_IntEnable(CMU_IEN_CALRDY);
  NVIC_ClearPendingIRQ(CMU_IRQn);
  NVIC_EnableIRQ(CMU_IRQn);

  calibrationState = HFRCO_CALIBRATION_IDLE;
}

static void resetCmuCalibrationFsm(void)
{
  calibrationState = HFRCO_CALIBRATION_OFF;
  tuned = false;
  finetuned = false;
  lastCalcUpcountGT = false;
  lastCalcUpcountLT = false;
  currentTuningVal = 0;
  newTuningVal = 0;
  previousUpcount = 0;
  tuningOver = &tuned;
}

static uint32_t computeTuning(uint32_t calculatedUpcount_a, uint32_t previousUpcount_a, uint32_t currentTuning_a, bool *tuned_a)
{
  uint32_t tuningVal = currentTuning_a;

  /*
   * If the up counter result is less than the tuned value, the LFRCO
   * is running at a lower frequency, so the tuning value has to be
   * decremented.
   */
  if (calculatedUpcount_a < theoreticalUpcount)
  {
    // Was the up counter greater than the tuned value on the last run?
    if (lastCalcUpcountGT)
    {
      /*
       * If the difference between the ideal count and the up count
       * from this run is greater than it was on the last run, then
       * the last run produced a more accurate tuning, so revert to
       * the previous tuning value.  If not, the current value gets
       * us the most accurate tuning.
       */
      if ((theoreticalUpcount - calculatedUpcount_a) > (previousUpcount_a - theoreticalUpcount))
        tuningVal = currentTuning_a;

      // Done tuning now
      *tuned_a = true;
    }
    // Up counter for the last run not greater than the tuned value
    else
    {
      /*
       * If the difference is 1, incrementing the tuning value again
       * will only increase the frequency further away from the
       * intended target, so tuning is now complete.
       */
      if ((theoreticalUpcount - calculatedUpcount_a) == 1)
        *tuned_a = true;
      /*
       * The difference between this run and the ideal count for the
       * desired frequency is > 1, so increase the tuning value to
       * increase the LFRCO frequency.  After the next calibration run,
       * the up counter value will increase.  Save the tuning value
       * from this run; if it's close, it might be more accurate than
       * the result from the next run.
       */
      else
      {
        //prevTuning = tuningVal;
        tuningVal--;
        lastCalcUpcountLT = true;  // Remember that the up counter was less than the ideal this run
        //prevUp = calculatedUpcount_a;
      }
    }
  }

  /*
   * If the up counter result is greater than the tuned value, the
   * HFRCO is running at a higher frequency, so the tuning value has
   * to be incremented.
   */
  if (calculatedUpcount_a > theoreticalUpcount)
  {
    // Was the up counter less than the tuned value on the last run?
    if (lastCalcUpcountLT)
    {
      /*
       * If the difference between the up count and the ideal count
       * from this run is greater than it was on the last run, then
       * the last run produced a more accurate tuning, so revert to
       * the previous tuning value.  If not, the current value gets
       * the most accurate tuning.
       */
      if ((calculatedUpcount_a - theoreticalUpcount) > (theoreticalUpcount - previousUpcount_a))
        tuningVal = currentTuning_a;

      // Done tuning now
      *tuned_a = true;
    }
    // Up counter for the last run not less than the tuned value
    else
    {
      /*
       * If the difference is 1, decrementing the tuning value again
       * will only decrease the frequency further away from the
       * intended target, so tuning is now complete.
       */
      if ((calculatedUpcount_a - theoreticalUpcount) == 1)
        *tuned_a = true;
      /*
       * The difference between this run and the ideal count for the
       * desired frequency is > 1, so decrease the tuning value to
       * decrease the LFRCO frequency.  After the next calibration run,
       * the up counter value will decrease.  Save the tuning value
       * from this run; if it's close, it might be more accurate than
       * the result from the next run.
       */
      else
      {
        //prevTuning = tuningVal;
        tuningVal++;
        lastCalcUpcountGT = true;  // Remember that the up counter was greater than the ideal this run
        //prevUp = calculatedUpcount_a;
      }
    }
  }

  return tuningVal;
}

void CMU_IRQHandler(void)
{
  uint32_t calculatedUpcount;

  // Clear the calibration ready flag
  CMU_IntClear(CMU_IF_CALRDY);

  // Get the up counter value
  calculatedUpcount = CMU_CalibrateCountGet();

  // Decision making based on result
  // Up counter result is equal to the desired value, end of calibration
  if (calculatedUpcount == theoreticalUpcount)
  {
    *tuningOver = true;
  }
  // Otherwise set new tuning value
  else
  {
    // Get New Tuning Value based on
    newTuningVal = computeTuning(calculatedUpcount, previousUpcount, currentTuningVal, tuningOver);
  }

  // If not tuned, run calibration again, otherwise stop
  if (*tuningOver)
  {
    CMU_CalibrateStop();
  } else {
    if(HFRCO_CALIBRATION_TUNING == calibrationState) {
      CMU_OscillatorTuningSet(cmuOsc_HFRCOEM23, newTuningVal);
    } else if(HFRCO_CALIBRATION_FINE_TUNING == calibrationState) {
      CMU_OscillatorFineTuningSet(cmuOsc_HFRCOEM23, newTuningVal);
    }
    previousUpcount = calculatedUpcount;
    currentTuningVal = newTuningVal;
  }
}


/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  initHFRCOEM23();
  initCmuCalibrationFsm();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  switch (calibrationState) {
    case HFRCO_CALIBRATION_IDLE:
      if(!tuned){
        // Get current tuningVal value
        currentTuningVal = CMU_OscillatorTuningGet(cmuOsc_HFRCOEM23);
        calibrationState = HFRCO_CALIBRATION_TUNING;
        // Start the up counter
        CMU_CalibrateStart();
      }
      break;
    case HFRCO_CALIBRATION_TUNING:
      if(tuned)
      {
        if(PERFORM_FINETUNE)
        {
          resetCmuCalibrationFsm();
          tuningOver = &finetuned;
          // Get current fine tuning value
          currentTuningVal = CMU_OscillatorFineTuningGet(cmuOsc_HFRCOEM23);
          calibrationState = HFRCO_CALIBRATION_FINE_TUNING;
          // Start the up counter
          CMU_CalibrateStart();
        } else {
          calibrationState = HFRCO_CALIBRATION_DONE;
        }
      }
      break;
    case HFRCO_CALIBRATION_FINE_TUNING:
      if(finetuned)
      {
        calibrationState = HFRCO_CALIBRATION_DONE;
      }
      break;
    case HFRCO_CALIBRATION_DONE:
      break;
    default:
      break;
  }

  if(calibrationState != HFRCO_CALIBRATION_DONE)
  {
    EMU_EnterEM1();
  } else {
    EMU_EnterEM2(true);
  }
}


//////////////////// EMLIB Extension //////////////////////////////////////

/***************************************************************************//**
 * @brief
 *   Get oscillator frequency fine tuning setting.
 *
 * @param[in] osc
 *   Oscillator to get fine tuning value for.
 *
 * @return
 *   The oscillator frequency fine tuning setting in use.
 ******************************************************************************/
uint32_t CMU_OscillatorFineTuningGet(CMU_Osc_TypeDef osc)
{
  uint32_t ret = 0U;

  switch (osc) {

    case cmuOsc_HFRCODPLL:
#if defined(CMU_CLKEN0_HFRCO0)
      CMU->CLKEN0_SET = CMU_CLKEN0_HFRCO0;
#endif
      ret = (HFRCO0->CAL & _HFRCO_CAL_FINETUNING_MASK) >> _HFRCO_CAL_FINETUNING_SHIFT;
      break;

#if defined(HFRCOEM23_PRESENT)
    case cmuOsc_HFRCOEM23:
      ret = (HFRCOEM23->CAL & _HFRCO_CAL_FINETUNING_MASK) >> _HFRCO_CAL_FINETUNING_SHIFT;
      break;
#endif

    default:
      EFM_ASSERT(false);
      break;
  }

  return ret;
}

/***************************************************************************//**
 * @brief
 *   Set the oscillator frequency finetuning control.
 *
 * @note
 *   Oscillator finetuning is done during production, and the finetuning value is
 *   automatically loaded after a reset. Changing the finetuning value from the
 *   calibrated value is for more advanced use. Certain oscillators also have
 *   build-in finetuning optimization.
 *
 * @param[in] osc
 *   Oscillator to set finetuning value for.
 *
 * @param[in] val
 *   The oscillator frequency finetuning setting to use.
 ******************************************************************************/
void CMU_OscillatorFineTuningSet(CMU_Osc_TypeDef osc, uint32_t val)
{
  switch (osc) {

    case cmuOsc_HFRCODPLL:
#if defined(CMU_CLKEN0_HFRCO0)
      CMU->CLKEN0_SET = CMU_CLKEN0_HFRCO0;
#endif
      EFM_ASSERT(val <= (_HFRCO_CAL_FINETUNING_MASK >> _HFRCO_CAL_FINETUNING_SHIFT));
      val &= _HFRCO_CAL_FINETUNING_MASK >> _HFRCO_CAL_FINETUNING_SHIFT;
      while ((HFRCO0->STATUS & HFRCO_STATUS_SYNCBUSY) != 0UL) {
      }
      HFRCO0->CAL = (HFRCO0->CAL & ~_HFRCO_CAL_FINETUNING_MASK)
                    | (val << _HFRCO_CAL_FINETUNING_SHIFT);
      break;

#if defined(HFRCOEM23_PRESENT)
    case cmuOsc_HFRCOEM23:
      EFM_ASSERT(val <= (_HFRCO_CAL_FINETUNING_MASK >> _HFRCO_CAL_FINETUNING_SHIFT));
      val &= _HFRCO_CAL_FINETUNING_MASK >> _HFRCO_CAL_FINETUNING_SHIFT;
      while ((HFRCOEM23->STATUS & HFRCO_STATUS_SYNCBUSY) != 0UL) {
      }
      HFRCOEM23->CAL = (HFRCOEM23->CAL & ~_HFRCO_CAL_FINETUNING_MASK)
                       | (val << _HFRCO_CAL_FINETUNING_SHIFT);
      break;
#endif

    default:
      EFM_ASSERT(false);
      break;
  }
}
