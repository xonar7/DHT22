// Decoder JavaScript para DHT22 (temperatura + humedad) vía LoRaWAN
// Para aplicación "dht22ap" en TTN
// Versión 1.0 - Sensor DHT22

function decodeUplink(input) {
    // Obtener los bytes recibidos
    var bytes = input.bytes;
    
    // Verificar que hay datos
    if (!bytes || bytes.length === 0) {
        return {
            errors: ["No hay datos en el payload"]
        };
    }
    
    // Verificar formato básico (esperamos 6 bytes)
    if (bytes.length < 6) {
        return {
            data: {
                raw: bytes,
                warning: "Payload menor a 6 bytes"
            }
        };
    }
    
    // ==================== TEMPERATURA ====================
    // Reconstruir el valor de temperatura de 16 bits (MSB, LSB)
    var tempRaw = (bytes[0] << 8) | bytes[1];
    
    // Determinar si el valor es negativo (complemento a 2)
    if (tempRaw & 0x8000) {
        tempRaw = -(0x10000 - tempRaw);
    }
    
    // Convertir a valor decimal (se dividió por 100 al enviar)
    var temperature = tempRaw / 100;
    temperature = parseFloat(temperature.toFixed(1));
    
    // ==================== HUMEDAD ====================
    // Reconstruir el valor de humedad de 16 bits (MSB, LSB)
    var humRaw = (bytes[2] << 8) | bytes[3];
    
    // Convertir a valor decimal
    var humidity = humRaw / 100;
    humidity = parseFloat(humidity.toFixed(1));
    
    // ==================== STATUS BYTE ====================
    var statusByte = bytes[4];
    var sensorValid = (statusByte & 0x01) !== 0;
    
    // ==================== IDENTIFICADOR ====================
    var sensorType = bytes[5];
    var sensorName = "";
    
    if (sensorType === 0x22) {
        sensorName = "DHT22";
    } else {
        sensorName = "Unknown";
    }
    
    // ==================== VALIDACIÓN DHT22 ====================
    // Rango válido DHT22: Temp: -40°C a +80°C, Hum: 0% a 100%
    var tempInRange = (temperature >= -40 && temperature <= 80);
    var humInRange = (humidity >= 0 && humidity <= 100);
    
    // Si el sensor marca error (-999), considerarlo inválido
    if (temperature < -900 || humidity < -900) {
        sensorValid = false;
        tempInRange = false;
        humInRange = false;
    }
    
    // ==================== CONSTRUIR RESPUESTA ====================
    var warnings = [];
    
    if (!sensorValid || !tempInRange) {
        warnings.push("Temperatura: Error o fuera de rango");
    }
    if (!sensorValid || !humInRange) {
        warnings.push("Humedad: Error o fuera de rango");
    }
    
    return {
        data: {
            // Datos principales
            temperature: temperature,
            temperature_valid: sensorValid && tempInRange,
            temperature_status: (sensorValid && tempInRange) ? "OK" : "ERROR",
            
            humidity: humidity,
            humidity_valid: sensorValid && humInRange,
            humidity_status: (sensorValid && humInRange) ? "OK" : "ERROR",
            
            // Información general
            temperature_unit: "°C",
            humidity_unit: "%",
            sensor_type: sensorName,
            sensor_id: sensorType,
            
            // Metadata
            bytes_received: bytes.length,
            status_byte: statusByte,
            
            // Información de red (si está disponible)
            rssi: input.metadata ? input.metadata.rssi : null,
            snr: input.metadata ? input.metadata.snr : null
        },
        warnings: warnings,
        errors: []
    };
}