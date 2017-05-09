// Bridge between main.c and C++ world

#include <stdint.h>
#include <stdbool.h>
#include <sstream>
#include <zxing/common/FastHybridBinarizer.h>
#include <zxing/common/HybridBinarizer.h>
#include <zxing/common/Counted.h>
#include <zxing/Binarizer.h>
#include <zxing/qrcode/QRCodeReader.h>
#include <zxing/Result.h>
#include <zxing/ReaderException.h>
#include <zxing/BinaryBitmap.h>
#include <zxing/DecodeHints.h>

#include <zxing/oned/OneDReader.h>
#include <zxing/oned/EAN13Reader.h>
#include <zxing/oned/EAN8Reader.h>
#include <zxing/oned/MultiFormatOneDReader.h>
#include <zxing/multi/GenericMultipleBarcodeReader.h>
#include <zxing/MultiFormatReader.h>



using namespace std;
using namespace zxing;

FastHybridBinarizer *binarizer;
Ref<BitMatrix> binary;

bool more = true;
int width_;
int height_;

#if 0
int KinetisVerify(void)
{
    int ret;
    volatile uint32_t ftfx_fstat;
    
    ftfx_fstat = *((volatile uint32_t*)0x40020000);
    ftfx_fstat = 0xFF;
    return 0;
}
#endif
extern "C" int cppInit(int width, int height) {
  width_ = width;
  height_ = height;
#if 0  
  if(KinetisVerify())
  {
    return 1;
  }
#endif  
  binarizer = new FastHybridBinarizer(width, height);
  binary = new BitMatrix(width_, height_);
  return 0;
}

extern "C" void cppResetBinarizer(void) {
  binarizer->reset();
}

extern "C" void cppProcessBlockRow(uint8_t *blockRow) {
  binarizer->writeBlockRow(blockRow);
}

extern "C" uint8_t *cppGetResults(void) {
  return binarizer->getResults();
}

vector<Ref<Result> > decode(Ref<BitMatrix> image, DecodeHints hints, uint32_t option) {
 
    Ref<qrcode::QRCodeReader> QRDreader(new qrcode::QRCodeReader);
    Ref<oned::MultiFormatOneDReader> OneReader(new oned::MultiFormatOneDReader(hints));
    if(option == 0)
    {
        return vector<Ref<Result> >(1, QRDreader->decode(binary, hints));

    }
    else
    {
        return vector<Ref<Result> >(1, OneReader->decode(binary, hints));
    }
}

extern "C" void set_region(int left, int top, int width, int height)
{
    binary->setRegion(left, top, width, height);
}

extern "C" bool get_pixel(int x, int y)
{
  return binary->get(x, y);
}

extern "C" void set_martix_data(int left, int top, int width, int height, uint8_t *data)
{
    binary->setSize(width, height);
    data += ((width*top) >>3);
    binary->setBits((int *)data);
}

extern "C" int read_image(uint32_t option, char *format, char *result)
{
  vector<Ref<Result> > results;
  string cell_result;
  int res = -1;

 try {
    DecodeHints hints(DecodeHints::DEFAULT_HINT);
    hints.setTryHarder(false);
  //  binary->setBits((int *)binarizer->getResults());
    results = decode(binary, hints, option);
    res = 0;
  } catch (const ReaderException& e) {
    cell_result = "zxing::ReaderException: " + string(e.what());
    res = -2;
 //   printf("%d %s\r\n", res, e.what());
  } catch (const zxing::IllegalArgumentException& e) {
    cell_result = "zxing::IllegalArgumentException: " + string(e.what());
    res = -3;
 //   printf("%d %s\r\n", res, e.what());
  } catch (const zxing::Exception& e) {
    cell_result = "zxing::Exception: " + string(e.what());
    res = -4;
  //  printf("%d %s\r\n", res, e.what());
  } catch (const std::exception& e) {
    cell_result = "std::exception: " + string(e.what());
    res = -5;
    //printf("%d %s\r\n", res, e.what());
  }
    
    for (size_t i = 0; i < results.size(); i++)
    {
    stringstream s;
    char buf[512];

        if (more)
        {
            s << "Format: "
            << BarcodeFormat::barcodeFormatNames[results[i]->getBarcodeFormat()]
            << endl;
            s.get(buf, sizeof(buf));
            strcpy(format, buf);

            for (int j = 0; j < results[i]->getResultPoints()->size(); j++)
            {
                s.str("");
                s.clear();
                s << "Point[" << j <<  "]: "
                << results[i]->getResultPoints()[j]->getX() << " "
                << results[i]->getResultPoints()[j]->getY() << endl;
                s.get(buf, sizeof(buf));
            }
        }
    s.str("");
    s.clear();
    s << results[i]->getText()->getText() << endl;
    s.get(buf, sizeof(buf));
    strcpy(result, buf);
    }
  return res;
}
