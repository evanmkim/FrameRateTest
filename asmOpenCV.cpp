/**
   Simple example of applying Gaussian blur using OpenCV.
   Converts between OpenCV's cv::Mat and Qt's QImage using utility functions.
   Andy Maloney <asmaloney@gmail.com>
   https://asmaloney.com/2013/11/code/converting-between-cvmat-and-qimage-or-qpixmap
**/

#include <QCommandLineParser>
#include <QCoreApplication>
#include <QDebug>
#include <QFileInfo>
#include <QImageReader>
#include <QImageWriter>
#include <QMap>

#include "asmOpenCV.h"


static void  sImageBlur( const QString &inInputFile, const QString &inOutputFile );

// Simple map of enum value to string since C++ STILL doesn't provide this...
static QMap<QImage::Format, QString>   sImageFormatEnumMap = {
   { QImage::Format::Format_Invalid, "Format_Invalid" },
   { QImage::Format::Format_Mono, "Format_Mono" },
   { QImage::Format::Format_MonoLSB, "Format_MonoLSB" },
   { QImage::Format::Format_Indexed8, "Format_Indexed8" },
   { QImage::Format::Format_RGB32, "Format_RGB32" },
   { QImage::Format::Format_ARGB32, "Format_ARGB32" },
   { QImage::Format::Format_ARGB32_Premultiplied, "Format_ARGB32_Premultiplied" },
   { QImage::Format::Format_RGB16, "Format_RGB16" },
   { QImage::Format::Format_ARGB8565_Premultiplied, "Format_ARGB8565_Premultiplied" },
   { QImage::Format::Format_RGB666, "Format_RGB666" },
   { QImage::Format::Format_ARGB6666_Premultiplied, "Format_ARGB6666_Premultiplied" },
   { QImage::Format::Format_RGB555, "Format_RGB555" },
   { QImage::Format::Format_ARGB8555_Premultiplied, "Format_ARGB8555_Premultiplied" },
   { QImage::Format::Format_RGB888, "Format_RGB888" },
   { QImage::Format::Format_RGB444, "Format_RGB444" },
   { QImage::Format::Format_ARGB4444_Premultiplied, "Format_ARGB4444_Premultiplied" },
   { QImage::Format::Format_RGBX8888, "Format_RGBX8888" },
   { QImage::Format::Format_RGBA8888, "Format_RGBA8888" },
   { QImage::Format::Format_RGBA8888_Premultiplied, "Format_RGBA8888_Premultiplied" },
   { QImage::Format::Format_BGR30, "Format_BGR30" },
   { QImage::Format::Format_A2BGR30_Premultiplied, "Format_A2BGR30_Premultiplied" },
   { QImage::Format::Format_RGB30, "Format_RGB30" },
   { QImage::Format::Format_A2RGB30_Premultiplied, "Format_A2RGB30_Premultiplied" },
   { QImage::Format::Format_Alpha8, "Format_Alpha8" },
   { QImage::Format::Format_Grayscale8, "Format_Grayscale8" }
};

static QString sQImageFormatToStr( QImage::Format inFormat )
{
   return sImageFormatEnumMap[inFormat];
}

static QString sCVTypeToStr( int inType )
{
   QString str( "CV_" );

   switch ( CV_MAT_DEPTH( inType ) )
   {
      case CV_8U:    str += "8U"; break;
      case CV_8S:    str += "8S"; break;
      case CV_16U:   str += "16U"; break;
      case CV_16S:   str += "16S"; break;
      case CV_32S:   str += "32S"; break;
      case CV_32F:   str += "32F"; break;
      case CV_64F:   str += "64F"; break;
      default:       str += "User"; break;
   }

   str += QStringLiteral( "C%1" ).arg( QString::number( CV_MAT_CN( inType ) ) );

   return str;
}

//int  main( int argc, char **argv )
//{
//   QCoreApplication   app( argc, argv );

//   QCommandLineParser parser;

//   parser.setApplicationDescription( "Apply gaussian blur to a given image." );
//   parser.addHelpOption();

//   parser.addOptions({
//                        { "i", "Image file", "file", "./exampleImage.png" },
//                        { "o", "Output file", "file", "./blurred.png" },
//                     });

//   parser.process( app );

//   const QString  cInputFile = parser.value( "i" );
//   const QString  cOutputFile = parser.value( "o" );

//   qInfo() << "   input:" << cInputFile;
//   qInfo() << "  output:" << cOutputFile;

//   sImageBlur( cInputFile, cOutputFile );

//   return 0;
//}

void  sImageBlur( const QString &inInputFile, const QString &inOutputFile )
{
   QImageReader   reader( inInputFile );

   if ( !reader.canRead() )
   {
      qWarning() << "Could not read image" << inInputFile;
      return;
   }

   QImage   image = reader.read();

   if ( image == QImage() )
   {
      qWarning() << "Error reading image" << inInputFile;
      return;
   }

   qInfo().noquote() << "Input image format:" << sQImageFormatToStr( image.format() );

   // Convert QImage to a cvMat
   cv::Mat  cvMat = ASM::QImageToCvMat( image, false );

   qInfo().noquote() << "cv::Mat format:" << sCVTypeToStr( cvMat.type() );

   // ... do any required OpenCV processing
   cv::GaussianBlur( cvMat, cvMat, cv::Size( 11, 11 ), 0, 0, cv::BORDER_DEFAULT );

   // Convert back to QImage so we can write it out
   QImage   blurred = ASM::cvMatToQImage( cvMat );
   qInfo().noquote() << "blurred image format:" << sImageFormatEnumMap[blurred.format()];

   QImageWriter   writer( inOutputFile, "png" );

   writer.write( blurred );
}

