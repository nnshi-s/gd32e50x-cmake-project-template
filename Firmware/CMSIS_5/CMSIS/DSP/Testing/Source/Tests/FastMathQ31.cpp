#include "FastMathQ31.h"
#include <stdio.h>
#include "Error.h"
#include "Test.h"


#define SNR_THRESHOLD 100
/* 

Reference patterns are generated with
a double precision computation.

*/
#define ABS_ERROR ((q31_t)2200)

/*

The error bounds is 0.014 and it is big
but the test is really extreme with input values as small
as 2^-31 !

The error is clearly diverging for the very small values.

So, we have an error converging to 0.014 for outputs around -21.


*/
#define LOG_ABS_ERROR ((q31_t)30000000)

    void FastMathQ31::test_vlog_q31()
    {
        const q31_t *inp  = input.ptr();
        q31_t *outp  = output.ptr();

        //printf("Nb samples = %lu\n",ref.nbSamples());
        arm_vlog_q31(inp,outp,ref.nbSamples());
        //arm_vlog_q31(inp+124,outp+124,1);
        //printf("in = %08X\n",inp[124]);
        //printf("out = %08X\n",outp[124]);
    
        //ASSERT_SNR(ref,output,(float32_t)SNR_THRESHOLD);
        ASSERT_NEAR_EQ(ref,output,LOG_ABS_ERROR);
        ASSERT_EMPTY_TAIL(output);

    }

    void FastMathQ31::test_cos_q31()
    {
        const q31_t *inp  = input.ptr();
        q31_t *outp  = output.ptr();
        unsigned long i;

        for(i=0; i < ref.nbSamples(); i++)
        {
          outp[i]=arm_cos_q31(inp[i]);
        }

        ASSERT_SNR(ref,output,(float32_t)SNR_THRESHOLD);
        ASSERT_NEAR_EQ(ref,output,ABS_ERROR);

    }

    void FastMathQ31::test_sin_q31()
    {
        const q31_t *inp  = input.ptr();
        q31_t *outp  = output.ptr();
        unsigned long i;

        for(i=0; i < ref.nbSamples(); i++)
        {
          outp[i]=arm_sin_q31(inp[i]);
        }

        ASSERT_SNR(ref,output,(float32_t)SNR_THRESHOLD);
        ASSERT_NEAR_EQ(ref,output,ABS_ERROR);

    }

    void FastMathQ31::test_sqrt_q31()
    {
        const q31_t *inp  = input.ptr();
        q31_t *outp  = output.ptr();
        arm_status status;
        unsigned long i;

        for(i=0; i < ref.nbSamples(); i++)
        {
           status=arm_sqrt_q31(inp[i],&outp[i]);
           ASSERT_TRUE((status == ARM_MATH_SUCCESS) || ((inp[i] <= 0) && (status == ARM_MATH_ARGUMENT_ERROR)));
        }

        ASSERT_SNR(ref,output,(float32_t)SNR_THRESHOLD);
        ASSERT_NEAR_EQ(ref,output,ABS_ERROR);

    }

  
    void FastMathQ31::setUp(Testing::testID_t id,std::vector<Testing::param_t>& paramsArgs,Client::PatternMgr *mgr)
    {
        (void)paramsArgs;
        switch(id)
        {
            case FastMathQ31::TEST_COS_Q31_1:
            {
               input.reload(FastMathQ31::ANGLES1_Q31_ID,mgr);
               ref.reload(FastMathQ31::COS1_Q31_ID,mgr);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_SIN_Q31_2:
            {
               input.reload(FastMathQ31::ANGLES1_Q31_ID,mgr);
               ref.reload(FastMathQ31::SIN1_Q31_ID,mgr);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_SQRT_Q31_3:
            {
               input.reload(FastMathQ31::SQRTINPUT1_Q31_ID,mgr);
               ref.reload(FastMathQ31::SQRT1_Q31_ID,mgr);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_VLOG_Q31_4:
            {
               input.reload(FastMathQ31::LOGINPUT1_Q31_ID,mgr);
               ref.reload(FastMathQ31::LOG1_Q31_ID,mgr);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_VLOG_Q31_5:
            {
               input.reload(FastMathQ31::LOGINPUT1_Q31_ID,mgr,3);
               ref.reload(FastMathQ31::LOG1_Q31_ID,mgr,3);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_VLOG_Q31_6:
            {
               input.reload(FastMathQ31::LOGINPUT1_Q31_ID,mgr,8);
               ref.reload(FastMathQ31::LOG1_Q31_ID,mgr,8);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;

            case FastMathQ31::TEST_VLOG_Q31_7:
            {
               input.reload(FastMathQ31::LOGINPUT1_Q31_ID,mgr,11);
               ref.reload(FastMathQ31::LOG1_Q31_ID,mgr,11);
               output.create(ref.nbSamples(),FastMathQ31::OUT_Q31_ID,mgr);

            }
            break;
        }
        
    }

    void FastMathQ31::tearDown(Testing::testID_t id,Client::PatternMgr *mgr)
    {
      (void)id;
      output.dump(mgr);
      
    }
