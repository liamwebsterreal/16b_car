/*
 * classify.ino
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 *
 */

#define MIC_INPUT                   P6_0

#define SIZE                        2752
#define SIZE_AFTER_FILTER           172
#define ADC_TIMER_MS                0.35

/*---------------------------*/
/*      CODE BLOCK PCA1      */
/*---------------------------*/

// Enveloping and threshold constants
#define SNIPPET_SIZE                81
#define PRELENGTH                   1
#define THRESHOLD                   0.46871776245211594

#define EUCLIDEAN_THRESHOLD         0.05
#define LOUDNESS_THRESHOLD          400

/*---------------------------*/
/*      CODE BLOCK PCA2      */
/*---------------------------*/

float pca_vec1[SNIPPET_SIZE] = {0.0032256670509441896, 0.0011749012834498251, 0.02325814720096675, 0.022493321064263383, -0.03492819156985344, -0.07386438312312232, -0.10729244781352715, -0.11209576875252737, -0.1423882991762613, -0.13879846609062108, -0.1506807145885137, -0.09876937416676711, -0.10647062145386937, -0.06809502084926759, -0.10609788227010235, -0.09929784937506149, -0.11566915717610259, -0.1351327014005156, -0.09440771513253741, -0.08074156258717767, -0.01662158018265023, -0.014739583679477112, 0.00808952365647113, 0.003735280649702214, -0.00982673491560616, -0.02569226475612998, -0.04605119182421229, -0.05277262459975387, -0.055680289685398475, -0.06397040050758027, -0.06619299556932715, -0.04393515207939497, -0.043177276360133895, -0.010381699569849015, 0.003704357495339589, 0.05757072950806257, 0.08348366308636881, 0.11779932844675135, 0.14164696232534071, 0.16535240423107328, 0.16381252018421014, 0.1955134794751186, 0.18791919659961584, 0.2051703337524632, 0.19864945285732535, 0.20856345174831611, 0.18942172663349732, 0.19231207572728853, 0.16675581904774514, 0.18758043215725415, 0.17282176494240845, 0.18105077423441862, 0.16144543805640107, 0.1503672004235593, 0.13072122637823794, 0.12070896528502796, 0.09362900192166547, 0.07942900736938954, 0.06206883803613368, 0.04859875377916563, 0.03624274296053566, 0.019291278882903107, 0.0033406540559464985, -0.013168971226872135, -0.02297791267119674, -0.05321731375476559, -0.04948704743214979, -0.09147099665823309, -0.07375373638568898, -0.11110005235850567, -0.09975341822087254, -0.11873462573007028, -0.12188810679860837, -0.12963865016851678, -0.1354682429220053, -0.1265703757779008, -0.13917186444446922, -0.12726165643757684, -0.13348907998070925, -0.12602442028387895};
float pca_vec2[SNIPPET_SIZE] = {-0.002373271017897595, 0.004717339206721766, 0.001940337672456549, -0.010394223853445794, 0.04165723464049992, 0.025602650256602244, 0.040184753136728646, 0.07266933565085516, 0.0634791735557713, 0.12049697245756759, 0.10180260372147566, 0.16805053789191848, 0.2203817939297348, 0.23699905054213474, 0.27117824002288776, 0.2444810281660766, 0.21782604304285133, 0.19343679800021824, 0.15689921228239698, 0.13281228004342308, 0.063909265429425, 0.04315985230321048, 0.033499981241376174, 0.016848942693259834, -0.013713928328874641, -0.06663295933103612, -0.13609454036446234, -0.16569659132551795, -0.20783727877117458, -0.23049900240029492, -0.25807644537991836, -0.25460914572093674, -0.2586463765497239, -0.23588222033047498, -0.19869799988357392, -0.14516779296782475, -0.11843080454998922, -0.06442253977753847, -0.03151109253407092, 0.008629780805591191, 0.005769768570489856, 0.03308594418801518, 0.02564174747918425, 0.03210315457963012, 0.03513314745818021, 0.03264335630617107, 0.03045623887457236, 0.02275644656804157, 0.027511781673311406, 0.03247996077017392, 0.033651509018500586, 0.03756827823127675, 0.05327328156157473, 0.06313855670762616, 0.0482885125539837, 0.06287660293788291, 0.0443701212267714, -0.008954510819283407, 0.027548391392564906, -0.032324697832548675, 0.010280433551324715, -0.013343064100054432, 0.005461279022178362, 0.019296166152699294, -0.0046092220821738585, 0.020258254204923606, -0.021393212633601603, 0.02258226203950289, -0.0493183885135906, -0.00463683049066054, -0.06451898591746512, -0.029371952338733846, -0.07093397100831392, -0.058238038269667884, -0.08844541240201914, -0.07080045791525624, -0.08219919741934244, -0.08015843203397453, -0.06657030029103583, -0.06233551460728599};
float pca_vec3[SNIPPET_SIZE] = {0.008684246124163197, 0.00933049567910299, 0.00969776391667726, 0.0009196694019670826, 0.005726688771691926, 0.0758142372052879, 0.016968746559367393, 0.04961746699255197, 0.014666026535203507, 0.036310125160782275, 0.02706928149710371, -0.0013615485807220028, -0.038455474887848735, -0.03392398096848818, -0.08727946401853331, -0.10378682310601606, -0.14816009033290706, -0.1488197853163693, -0.1606004512893439, -0.20256153436842017, -0.16752543140284376, -0.22006629561477434, -0.19600440091596766, -0.2200083129929382, -0.18862583485965545, -0.21356327446872583, -0.16763798194805715, -0.17336609996761698, -0.14287454482542292, -0.1197556883594366, -0.11522290746606395, -0.07482653427699257, -0.07917846643087972, -0.0312994892624411, -0.05768209805791047, -0.025172496059171615, -0.07014189034706399, -0.047967810099604415, -0.10571814255875005, -0.08396400697395438, -0.11685987289760873, -0.06161982223690695, -0.09504024737374252, -0.04110047552787369, -0.03782535803428774, 0.0003242037847226991, -0.007563281747283045, 0.01571267811249383, 0.0425477807698088, 0.04082711512109425, 0.042697299671895436, 0.07922818998389117, 0.060927605244988164, 0.09358819810942492, 0.10804877457050172, 0.1068931639477828, 0.13865855553432918, 0.14020934269606866, 0.16301598017125912, 0.1615609864921304, 0.1865564548693242, 0.172211347499647, 0.1821627101026382, 0.1566451588790253, 0.14630978723683558, 0.13984171693362002, 0.11974464118752771, 0.11561532177026251, 0.09552748678629149, 0.10369262371519265, 0.08205048858093911, 0.09542909341860202, 0.08957474726286171, 0.10138934700863066, 0.08983102069183788, 0.10143821468835892, 0.0952805385782737, 0.0922567399991502, 0.08451263963638214, 0.08644521667493174};
float projected_mean_vec[3] = {-0.012732594015261455, 0.018066513790912793, -0.0005004573140877564};
float centroid1[3] = {0.021161231632266856, -0.008138187819506471, -0.04250909847620198};
float centroid2[3] = {0.048688010771901116, 0.012464417263297958, 0.021530192876460734};
float centroid3[3] = {-0.04530502378367618, 0.0447382690225393, 0.007520447079454712};
float centroid4[3] = {-0.024544218620491807, -0.04906449846633078, 0.013458458520286525};
float* centroids[4] = {
  (float *) &centroid1, (float *) &centroid2,
  (float *) &centroid3, (float *) &centroid4
};

/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

float result[SNIPPET_SIZE] = {0};
float proj1 = 0;
float proj2 = 0;
float proj3 = 0;

// Data array and index pointer
int re[SIZE] = {0};
volatile int re_pointer = 0;

/*---------------------------*/
/*       Norm functions      */
/*---------------------------*/

// Compute the L2 norm of (dim1, dim2) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        centroid: size-2 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm(float dim1, float dim2, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2));
}

// Compute the L2 norm of (dim1, dim2, dim3) and centroid
// input: dim1: 1st dimension coordinate
//        dim2: 2nd dimension coordinate
//        dim3: 3rd dimension coordinate
//        centroid: size-3 array containing centroid coordinates
// output: L2 norm (Euclidean distance) between point and centroid
float l2_norm3(float dim1, float dim2, float dim3, float* centroid) {
  return sqrt(pow(dim1-centroid[0],2) + pow(dim2-centroid[1],2) + pow(dim3-centroid[2],2));
}

void setup(void) {
  Serial.begin(38400);

  pinMode(MIC_INPUT, INPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  re_pointer = 0;
  reset_blinker();
  setTimer();
}

void loop(void) {
  if (re_pointer == SIZE) {
    digitalWrite(RED_LED, LOW);

    // Apply enveloping function and get snippet with speech.
    // Do classification only if loud enough.
    if (envelope(re, result)) {

      // Reset projection result variables declared above
      proj1 = 0;
      proj2 = 0;
      proj3 = 0;

      /*---------------------------*/
      /*      CODE BLOCK PCA3      */
      /*---------------------------*/

      // Project 'result' onto the principal components
      // Hint: 'result' is an array
      // Hint: do this entire operation in 1 loop by replacing the '...'
      // YOUR CODE HERE
      for (int i = 0; i < SNIPPET_SIZE; i++) {
          proj1 += result[i] * pca_vec1[i];
          proj2 += result[i] * pca_vec2[i];
          proj3 += result[i] * pca_vec3[i];
      }

      // Demean the projection
      proj1 -= projected_mean_vec[3];
      proj2 -= projected_mean_vec[3];
      proj3 -= projected_mean_vec[3];

      // Classification
      // Use the function 'l2_norm' defined above
      // ith centroid: 'centroids[i]'
      float best_dist = 999999;
      int best_index = -1;
      float cent[4];
      cent[0] = l2_norm3(proj1,proj2,proj3,centroids[0]);
      cent[1] = l2_norm3(proj1,proj2,proj3,centroids[1]);
      cent[2] = l2_norm3(proj1,proj2,proj3,centroids[2]);
      cent[3] = l2_norm3(proj1,proj2,proj3,centroids[3]);
      for(int index =0; index < 4; index++){
        if(cent[index] < best_dist){
          best_dist = cent[index];
          best_index = index;
        }
      }
      

      // Compare 'best_dist' against the 'EUCLIDEAN_THRESHOLD' and print the result
      // If 'best_dist' is less than the 'EUCLIDEAN_THRESHOLD', the recording is a word
      // Otherwise, the recording is noise
      if(best_dist >= EUCLIDEAN_THRESHOLD) {
        Serial.println("Error Threshold not met");
      } else {
        if(best_index == 0) {
          Serial.println("Car");
        } else if(best_index == 1) {
          Serial.println("index");
        } else if(best_index == 2) {
          Serial.println("population");
        } else if(best_index == 3) {
          Serial.println("additionality");
        } else {
          Serial.println("Something is wrong");
        }
      }

      /*---------------------------*/
      /*---------------------------*/
      /*---------------------------*/
    }
    else {
      Serial.println("Below LOUDNESS_THRESHOLD.");
    }

    delay(2000);
    re_pointer = 0;
  }
}

// Enveloping function with thresholding and normalizing,
// returns snippet of interest (containing speech)
bool envelope(int* data, float* data_out) {
  int32_t avg = 0;
  float maximum = 0;
  int32_t total = 0;
  int block;

  // Apply enveloping filter while finding maximum value
  for (block = 0; block < SIZE_AFTER_FILTER; block++) {
    avg = 0;
    for (int i = 0; i < 16; i++) {
      avg += data[i+block*16];
    }
    avg = avg >> 4;
    data[block] = abs(data[block*16] - avg);
    for (int i = 1; i < 16; i++) {
      data[block] += abs(data[i+block*16] - avg);
    }
    if (data[block] > maximum) {
      maximum = data[block];
    }
  }

  // If not loud enough, return false
  if (maximum < LOUDNESS_THRESHOLD) {
    return false;
  }

  // Determine threshold
  float thres = THRESHOLD * maximum;

  // Figure out when interesting snippet starts and write to data_out
  block = PRELENGTH;
  while (data[block++] < thres);
  if (block > SIZE_AFTER_FILTER - SNIPPET_SIZE) {
    block = SIZE_AFTER_FILTER - SNIPPET_SIZE;
  }
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data[block-PRELENGTH+i];
    total += data_out[i];
  }

  // Normalize data_out
  for (int i = 0; i < SNIPPET_SIZE; i++) {
    data_out[i] = data_out[i] / total;
  }

  return true;
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void reset_blinker(void) {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  delay(100);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
  delay(100);
  digitalWrite(GREEN_LED, LOW);
}

/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

// ISR for timestep
#pragma vector=TIMER2_A0_VECTOR    // Timer A ISR
__interrupt void Timer2_A0_ISR(void) {
  if (re_pointer < SIZE) {
    digitalWrite(RED_LED, HIGH);
    re[re_pointer] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;
  }
}

// Set timer for timestep; use A2 since A0 & A1 are used by PWM
void setTimer(void) {
  // Set the timer based on 25MHz clock
  TA2CCR0 = (unsigned int) (25000*ADC_TIMER_MS);
  TA2CCTL0 = CCIE;
  __bis_SR_register(GIE);
  TA2CTL = TASSEL_2 + MC_1 + TACLR + ID_0;
}
