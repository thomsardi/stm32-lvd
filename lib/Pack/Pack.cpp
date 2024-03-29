#include <Pack.h>

/**
 * @brief   class constructor, takes an array as storage of batterydata
 *          @note   this method take an array with fixed size to declare its class vector, preventing the dynamic allocation
 * @param   batteryData an array of BatteryData data type, this array is a storage for vector
 * @param   maxSize define the max size of the storage array
 * @param   size    the actual position index of the vector, default 0 because the vector still empty
*/
Pack::Pack(BatteryData batteryData[], size_t maxSize, size_t size)
{
    stack.setStorage(batteryData, maxSize, size);
}

/**
 * @brief   print all the vector element data
*/
void Pack::printInfo()
{
    for (size_t i = 0; i < stack.size(); i++)
    {
        Serial1.println("===================Begin====================");
        Serial1.println("Prev Pack Counter : " + String(stack.at(i).cnt.previousPackUpdatedCounter));
        Serial1.println("Pack Counter : " + String(stack.at(i).cnt.packUpdatedCounter));
        Serial1.println("Prev Mosfet Counter : " + String(stack.at(i).cnt.previousMosfetUpdatedCounter));
        Serial1.println("Mosfet Counter : " + String(stack.at(i).cnt.mosfetUpdatedCounter));
        Serial1.println("Id : " + String(stack.at(i).id));
        Serial1.println("Pack Voltage : " + String(stack.at(i).packVoltage));
        Serial1.println("Pack Current : " + String(stack.at(i).packCurrent));
        Serial1.println("Pack SoC : " + String(stack.at(i).packSoc));
        Serial1.println("CMOS : " + String(stack.at(i).mosfetStatus.cmos));
        Serial1.println("DMOS : " + String(stack.at(i).mosfetStatus.dmos));
        Serial1.println("Top Temp : " + String(stack.at(i).temperature.top));
        Serial1.println("Mid Temp : " + String(stack.at(i).temperature.mid));
        Serial1.println("Bot Temp : " + String(stack.at(i).temperature.bot));
        Serial1.println("CMOS Temp : " + String(stack.at(i).temperature.dmosTemp));
        Serial1.println("DMOS Temp : " + String(stack.at(i).temperature.cmosTemp));
        Serial1.println("===================End====================");
    }
}

/**
 * @brief   insert data into vector
 *          @note   this will first find an existing data, if found it will overwrite the existing data, if not
 *                  the data will be pushed back into vector as a new data element
 * @param   data    struct of HalfPackData
*/
void Pack::insert(const HalfPackData &data)
{
    for (size_t i = 0; i < stack.size(); i++)
    {
        if(stack.at(i).id == data.id)
        {
            stack.at(i).isUpdated = data.isUpdated;
            stack.at(i).packVoltage = data.packVoltage;
            stack.at(i).packCurrent = data.packCurrent;
            stack.at(i).packSoc = data.packSoc;
            stack.at(i).incPackCounter();
            return;
        }
    }
    if(stack.size() >= stack.max_size())
    {
        return;
    }
    BatteryData dummy;
    dummy.id = data.id;
    dummy.isUpdated = data.isUpdated;
    dummy.packVoltage = data.packVoltage;
    dummy.packCurrent = data.packCurrent;
    dummy.packSoc = data.packSoc;
    dummy.incPackCounter();
    stack.push_back(dummy);
}

/**
 * @brief   insert data into vector
 *          @note   this will first find an existing data, if found it will overwrite the existing data, if not
 *                  the data will be pushed back into vector as a new data element
 * @param   data    struct of HalfMosfetData
*/          
void Pack::insert(const HalfMosfetData &data)
{
    for (size_t i = 0; i < stack.size(); i++)
    {
        if(stack.at(i).id == data.id)
        {
            stack.at(i).isUpdated = data.isUpdated;
            stack.at(i).mosfetStatus.val = data.mosfetStatus.val;
            stack.at(i).temperature.top = data.temperature.top;
            stack.at(i).temperature.mid = data.temperature.mid;
            stack.at(i).temperature.bot = data.temperature.bot;
            stack.at(i).temperature.cmosTemp = data.temperature.cmosTemp;
            stack.at(i).temperature.dmosTemp = data.temperature.dmosTemp;
            stack.at(i).isCmosOverTemperature = data.isCmosOverTemperature;
            stack.at(i).isDmosOverTemperature = data.isDmosOverTemperature;
            stack.at(i).incMosfetCounter();
            return;
        }
    }
    if(stack.size() >= stack.max_size())
    {
        return;
    }
    BatteryData dummy;
    dummy.id = data.id;
    dummy.isUpdated = data.isUpdated;
    dummy.mosfetStatus.val = data.mosfetStatus.val;
    dummy.temperature.top = data.temperature.top;
    dummy.temperature.mid = data.temperature.mid;
    dummy.temperature.bot = data.temperature.bot;
    dummy.temperature.cmosTemp = data.temperature.cmosTemp;
    dummy.temperature.dmosTemp = data.temperature.dmosTemp;
    dummy.isCmosOverTemperature = data.isCmosOverTemperature;
    dummy.isDmosOverTemperature = data.isDmosOverTemperature;
    dummy.incMosfetCounter();
    stack.push_back(dummy);
}

/**
 * @brief   remove data with contained id
 * @param   id  id of the data
*/
void Pack::remove(const int id)
{
    for (size_t i = 0; i < stack.size(); i++)
    {
        if(stack.at(i).id == id)
        {
            stack.remove(i);
            return;
        }
    }
}

/**
 * @brief   Method to clean up data that was not updated periodically
 *          @note call this method periodically in loop, use millis to trigger cleanup every x second(s)
*/
void Pack::removeUnusedData()
{
    for (size_t i = 0; i < stack.size(); i++)
    {
      if (stack.at(i).isUpdated)
      {
        if(stack.at(i).getLastPackCounter() == stack.at(i).getPackCounter())
        {
          stack.at(i).isUpdated = 0;
          stack.remove(i);
        }
        else
        {
            stack.at(i).updatePackPreviousCounter();
            stack.at(i).updateMosfetPreviousCounter();
        }
      }
    }
}

/**
 * @brief   get data at stack index
 * @param   index  index of stack
 * @return  BatteryData refer to DataDef.h for the struct member, return default BatteryData if not found
*/
BatteryData Pack::getDataByIndex(const int index)
{
    BatteryData data;
    data.id = 0;
    if (index >= stack.size())
    {
        return data;
    }
    return stack.at(index);
}

/**
 * @brief   get data with contained id
 * @param   id  id to search
 * @return  BatteryData refer to DataDef.h for the struct member, return default BatteryData if not found
*/
BatteryData Pack::getDataById(const int id)
{
    BatteryData data;
    data.id = 0;
    
    for (size_t i = 0; i < stack.size(); i++)
    {
        if (stack.at(i).id == id)
        {
            return stack.at(i);
        }
    }
    
    return data;
}


/**
 * @brief   get data with contained id
 * @param   id  id to search
 * @param   batData destination address
*/
void Pack::getDataById(const int id, BatteryData &batData)
{
    BatteryData data;
    data.id = 0;
    
    for (size_t i = 0; i < stack.size(); i++)
    {
        if (stack.at(i).id == id)
        {
            batData.id = stack.at(i).id;
            batData.packVoltage = stack.at(i).packVoltage;
            batData.packCurrent = stack.at(i).packCurrent;
            batData.packSoc = stack.at(i).packSoc;
            batData.isUpdated = stack.at(i).isUpdated;
            batData.isCmosOverTemperature = stack.at(i).isCmosOverTemperature;
            batData.isDmosOverTemperature = stack.at(i).isDmosOverTemperature;
            batData.mosfetStatus.val = stack.at(i).mosfetStatus.val;
            batData.cnt.previousTemperatureUpdatedCounter = stack.at(i).cnt.previousTemperatureUpdatedCounter;
            batData.cnt.temperatureUpdatedCounter = stack.at(i).cnt.temperatureUpdatedCounter;
            batData.cnt.previousMosfetUpdatedCounter = stack.at(i).cnt.previousMosfetUpdatedCounter;
            batData.cnt.mosfetUpdatedCounter = stack.at(i).cnt.mosfetUpdatedCounter;
            batData.cnt.previousPackUpdatedCounter = stack.at(i).cnt.previousPackUpdatedCounter;
            batData.cnt.packUpdatedCounter = stack.at(i).cnt.packUpdatedCounter;
            return;
        }
    }
    batData.id = 0;
}

            

/**
 * @brief   calculate voltage, current, SoC, total, inactive and active of battery
*/
void Pack::calculate()
{
    int activeBattery = 0;
    int totalVoltage = 0;
    int totalCurrent = 0;
    int totalSoc = 0;
    int32_t averageVoltage = 0;
    int averageSoc = 0;

    // Serial1.println("Stack size : " + String(stack.size()));
    for (size_t i = 0; i < stack.size(); i++)
    {
        if(stack.at(i).isUpdated && stack.at(i).mosfetStatus.dmos)
        {
            totalVoltage += stack.at(i).packVoltage;
            totalCurrent += stack.at(i).packCurrent;
            totalSoc += stack.at(i).packSoc;
            activeBattery++;
        }
    }
    _totalBattery = stack.size();
    _activeBattery = activeBattery;
    _inactiveBattery = _totalBattery - _activeBattery;
    averageVoltage = totalVoltage / activeBattery;
    averageSoc = totalSoc / activeBattery;
    _averageVoltage = averageVoltage;
    _totalCurrent = totalCurrent;
    _averageSoc = averageSoc;
}

/**
 * @brief   get an average voltage of battery
 *          @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  average voltage
*/
int32_t Pack::getAverageVoltage() {
    return _averageVoltage;
}

/**
 * @brief   get an total current of battery
 *          @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  total current
*/
int Pack::getTotalCurrent() {
    return _totalCurrent;
}

/**
 * @brief   get an average SoC of battery
 *          @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  average SoC
*/
int Pack::getAverageSoc() {
    return _averageSoc;
}

/**
 * @brief   get total battery
 *          @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  total battery
*/
uint8_t Pack::getTotalBattery() {
    return _totalBattery;
}

/**
 * @brief   get a total active battery
 *          @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  total active battery
*/
uint8_t Pack::getActiveBattery() {
    return _activeBattery;
}

/**
 * @brief   get an inactive battery
*           @note   call the calculate() method before using this method to ensure that the data is properly updated
 * @return  total inactive battery
*/
uint8_t Pack::getInactiveBattery() {
    return _inactiveBattery;
}

Pack::~Pack()
{

}