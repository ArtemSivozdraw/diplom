package main

import (
        "fmt"
        "strings"
)

// handleCommand обробляє вхідні TCP-команди від клієнтів
func (s *Server) handleCommand(client *Client, command string) {
        parts := strings.Split(command, " ")
        action := parts[0]

        switch action {
        case "WATCH_VIDEO":
                if len(parts) < 2 {
                        client.TCPConn.Write([]byte("ERROR: Missing target ID\n"))
                        return
                }
                targetID := strings.TrimSpace(parts[1])

                s.mutex.Lock()
                client.WatchingID = targetID
                s.mutex.Unlock()

                fmt.Printf("[Routing] %s watching video %s\n", client.ID, targetID)
                client.TCPConn.Write([]byte("OK: Subscribed\n"))

        default:
                client.TCPConn.Write([]byte("ERROR: Unknown command\n"))
        }
}

// routeUDPData відповідає за двосторонню пересилку пакетів
func (s *Server) routeUDPData(senderID string, payload []byte) {
        s.mutex.RLock()
        defer s.mutex.RUnlock()

        senderClient, exists := s.clients[senderID]
        if !exists {
                return
        }

        strPayload := string(payload)


        // Якщо це пакет керування від пілота (наприклад, "CTRL:...")
        if strings.HasPrefix(strPayload, "CTRL:") && senderClient.WatchingID != "" {
                targetDroneID := senderClient.WatchingID
                if drone, ok := s.clients[targetDroneID]; ok && drone.UDPAddr != nil {
                // Пересилаємо команди керування дрону
                s.udpConn.WriteToUDP(payload, drone.UDPAddr)
                }
                return
        }

        // ЛОГІКА 1: Телеметрія (ACK від Пілота до Дрона)
        // Якщо це підтвердження і відправник зараз є глядачем
        if strings.HasPrefix(strPayload, "ACK:") && senderClient.WatchingID != "" {
                targetDroneID := senderClient.WatchingID
                if drone, ok := s.clients[targetDroneID]; ok && drone.UDPAddr != nil {
                        // Пересилаємо ACK дрону
                        s.udpConn.WriteToUDP(payload, drone.UDPAddr)
                }
                return // Завершуємо, щоб не розсилати ACK як відео
        }

        // ЛОГІКА 2: Відео (Від Дрона до всіх Пілотів)
        // Якщо це не ACK, значить це JPEG-кадр від дрона. Розсилаємо підписникам.
        for _, client := range s.clients {
                if client.WatchingID == senderID && client.UDPAddr != nil {
                        s.udpConn.WriteToUDP(payload, client.UDPAddr)
                }
        }
}