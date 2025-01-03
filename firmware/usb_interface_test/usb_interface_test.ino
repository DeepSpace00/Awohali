#include <Wire.h>
#include "SparkFun_USB251x_Arduino_Library.h"   //Click here to install: http://librarymanager/All#USB251x
#include <vector>
#include <string>
//#include "UsbHostMsd.h"
#include <Arduino_USBHostMbed5.h>
#include "FATFileSystem.h"

#define TEST_FS_NAME "USB"
#define TEST_FOLDER_NAME "TEST_FOLDER"
#define TEST_FILE "test.txt"
#define DELETE_FILE_DIMENSION 150

USB251x myHub;
USBHostMSD block_device;
mbed::FATFileSystem fs(TEST_FS_NAME);

std::string root_folder = std::string("/") + std::string(TEST_FS_NAME);
std::string folder_test_name = root_folder + std::string("/") + std::string(TEST_FOLDER_NAME);
std::string file_test_name = folder_test_name + std::string("/") + std::string(TEST_FILE);

/* this callback will be called when a Mass Storage Device is plugged in */
void device_attached_callback(void) {
  Serial.println();
  Serial.println("++++ Mass Storage Device detected ++++");
  Serial.println();
}

void setup() {
  /*
   *  SERIAL INITIALIZATION
   */
  Serial.begin(115200);
  while (!Serial) {
  }

  Wire.begin();

  if (myHub.begin() == false) {
    Serial.println("Device not found. USB251xB may already be in hub mode. Please check wiring or reset the hub. Freezing...");
    while (1)
      ;
  }

  Serial.println("Writing default settings to hub");
  myHub.setDefaults();  //Write ROM defaults
  myHub.attach();       //Locks settings and begin acting as hub


  Serial.println();
  Serial.println("*** USB HOST Mass Storage Device example ***");
  Serial.println();

  /* attach the callback so that when the device is inserted the device_attached_callback
     will be automatically called */
  block_device.attach_detected_callback(device_attached_callback);
  /* list to store all directory in the root */
  std::vector<std::string> dir_list;

  /* 
   *  Check for device to be connected
   */

  int count = 0;
  while (!block_device.connect()) {
    if (count == 0) {
      Serial.println("Waiting for Mass Storage Device");
    } else {
      Serial.print(".");
      if (count % 30 == 0) {
        Serial.println();
      }
    }
    count++;
    delay(1000);
  }

  Serial.println("Mass Storage Device connected.");

  /* 
   *  MOUNTING SDCARD AS FATFS filesystem
   */

  Serial.println("Mounting Mass Storage Device...");
  int err = fs.mount(&block_device);
  if (err) {
    // Reformat if we can't mount the filesystem
    // this should only happen on the first boot
    Serial.println("No filesystem found, formatting... ");
    err = fs.reformat(&block_device);
  }

  if (err) {
    Serial.println("Error formatting USB Mass Storage Device");
    while (1)
      ;
  }

  /* 
   *  READING root folder
   */

  DIR *dir;
  struct dirent *ent;
  int dirIndex = 0;

  Serial.println("*** List USB Mass Storage Device content: ");
  if ((dir = opendir(root_folder.c_str())) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      if (ent->d_type == DT_REG) {
        Serial.print("- [File]: ");
      } else if (ent->d_type == DT_DIR) {
        Serial.print("- [Fold]: ");
        if (ent->d_name[0] != '.') { /* avoid hidden folders (.Trash might contain a lot of files) */
          dir_list.push_back(ent->d_name);
        }
      }
      Serial.println(ent->d_name);
      dirIndex++;
    }
    closedir(dir);
  } else {
    // Could not open directory
    Serial.println("Error opening USB Mass Storage Device\n");
    while (1)
      ;
  }

  if (dirIndex == 0) {
    Serial.println("Empty SDCARD");
  }

  bool found_test_folder = false;

  /* 
   *  LISTING CONTENT of the first level folders (the one immediately present in root folder)
   */

  if (dir_list.size()) {
    Serial.println();
    Serial.println("Listing content of folders in root: ");
  }
  for (unsigned int i = 0; i < dir_list.size(); i++) {
    if (dir_list[i] == TEST_FOLDER_NAME) {
      found_test_folder = true;
    }
    Serial.print("- ");
    Serial.print(dir_list[i].c_str());
    Serial.println(":");

    std::string d = root_folder + std::string("/") + dir_list[i];
    if ((dir = opendir(d.c_str())) != NULL) {
      while ((ent = readdir(dir)) != NULL) {
        if (ent->d_type == DT_REG) {
          Serial.print("   - [File]: ");
        } else if (ent->d_type == DT_DIR) {
          Serial.print("   - [Fold]: ");
        }
        Serial.println(ent->d_name);
      }
      closedir(dir);
    } else {
      Serial.print("ERROR OPENING SUB-FOLDER ");
      Serial.println(d.c_str());
    }
  }

  /* 
   *  CREATING TEST FOLDER (if does not exist already)
   */

  err = 0;
  if (!found_test_folder) {
    Serial.println("TEST FOLDER NOT FOUND... creating folder test");
    err = mkdir(folder_test_name.c_str(), S_IRWXU | S_IRWXG | S_IRWXO);
    if (err != 0) {
      Serial.print("FAILED folder creation with error ");
      Serial.println(err);
    }
  }

  /* 
   *  READING TEST FILE CONTENT
   */

  if (err == 0) {
    int file_dimension = 0;
    FILE *fp = fopen(file_test_name.c_str(), "r");
    if (fp != NULL) {
      Serial.print("Opened file: ");
      Serial.print(file_test_name.c_str());
      Serial.println(" for reading");

      fseek(fp, 0L, SEEK_END);
      int numbytes = ftell(fp);
      fseek(fp, 0L, SEEK_SET);

      Serial.print("Bytes in the file: ");
      Serial.println(numbytes);
      file_dimension = numbytes;

      if (numbytes > 0) {
        Serial.println();
        Serial.println("-------------------- START FILE CONTENT --------------------");
      }

      for (int i = 0; i < numbytes; i++) {
        char ch;
        fread(&ch, sizeof(char), 1, fp);
        Serial.print(ch);
      }

      if (numbytes > 0) {
        Serial.println("--------------------- END FILE CONTENT ---------------------");
        Serial.println();
      } else {
        Serial.println("File is EMPTY!");
        Serial.println();
      }

      fclose(fp);
    } else {
      Serial.print("FAILED open file ");
      Serial.println(file_test_name.c_str());
    }

    /*
     * DELETE FILE IF THE File dimension is greater than 150 bytes
     */

    if (file_dimension > DELETE_FILE_DIMENSION) {
      Serial.println("Test file reached the delete dimension... deleting it!");
      if (remove(file_test_name.c_str()) == 0) {
        Serial.println("TEST FILE HAS BEEN DELETED!");
      }
    }

    /*
     * APPENDING SOMETHING TO FILE 
     */

    fp = fopen(file_test_name.c_str(), "a");
    if (fp != NULL) {
      Serial.print("Opened file: ");
      Serial.print(file_test_name.c_str());
      Serial.println(" for writing (append)");
      char text[] = "This line has been appended to file!\n";
      fwrite(text, sizeof(char), strlen(text), fp);
      fclose(fp);
    } else {
      Serial.print("FAILED open file for appending ");
      Serial.println(file_test_name.c_str());
    }

    /*
     * READING AGAIN FILE CONTENT
     */

    fp = fopen(file_test_name.c_str(), "r");
    if (fp != NULL) {
      Serial.print("Opened file: ");
      Serial.print(file_test_name.c_str());
      Serial.println(" for reading");

      fseek(fp, 0L, SEEK_END);
      int numbytes = ftell(fp);
      fseek(fp, 0L, SEEK_SET);

      Serial.print("Bytes in the file: ");
      Serial.println(numbytes);

      if (numbytes > 0) {
        Serial.println();
        Serial.println("-------------------- START FILE CONTENT --------------------");
      }

      for (int i = 0; i < numbytes; i++) {
        char ch;
        fread(&ch, sizeof(char), 1, fp);
        Serial.print(ch);
      }

      if (numbytes > 0) {
        Serial.println("--------------------- END FILE CONTENT ---------------------");
        Serial.println();
      } else {
        Serial.println("File is EMPTY!");
        Serial.println();
      }

      fclose(fp);

    } else {
      Serial.print("FAILED open file for appending ");
      Serial.println(file_test_name.c_str());
    }
  }
}

void loop() {
  // Empty
}