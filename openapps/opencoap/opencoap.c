#include "opendefs.h"
#include "opencoap.h"
#include "openqueue.h"
#include "openserial.h"
#include "openrandom.h"
#include "packetfunctions.h"
#include "idmanager.h"
#include "opentimers.h"
#include "scheduler.h"

//=========================== defines =========================================

//=========================== variables =======================================

opencoap_vars_t opencoap_vars;

//=========================== prototype =======================================
//=========================== public ==========================================

//===== from stack

/**
\brief Initialize this module.
*/
void opencoap_init() {
   // initialize the resource linked list
   opencoap_vars.resources     = NULL;
   
   // initialize the messageID
   opencoap_vars.messageID     = openrandom_get16b();

   // register at UDP stack
   opencoap_vars.desc.port              = WKP_UDP_COAP;
   opencoap_vars.desc.callbackReceive   = &opencoap_receive;
   opencoap_vars.desc.callbackSendDone  = opencoap_sendDone;
   openudp_register(&opencoap_vars.desc);
}

/**
\brief Indicate a CoAP messages was received.

A "CoAP message" is simply a UDP datagram received on the CoAP UDP port.

This function will call the appropriate resource, and send back its answer. The
received packetbuffer is reused to contain the response (or error code).

\param[in] msg The received CoAP message.
*/
void opencoap_receive(OpenQueueEntry_t* msg) {
   uint16_t                  temp_l4_destination_port;
   uint8_t                   index;
   coap_resource_desc_t*     temp_desc;
   bool                      found;
   owerror_t                 outcome = 0;
   coap_type_t               response_type;
   // local variables passed to the handlers (with msg)
   coap_header_iht           coap_header;
   coap_option_iht           coap_incomingOptions[MAX_COAP_OPTIONS];
   coap_option_iht           coap_outgoingOptions[MAX_COAP_OPTIONS];
   uint8_t                   coap_incomingOptionsLen;
   uint8_t                   coap_outgoingOptionsLen;
   owerror_t                 decStatus;
   coap_option_iht*          objectSecurity;
   uint16_t                  rcvdSequenceNumber;
   uint8_t*                  rcvdKid;
   uint8_t                   rcvdKidLen;
    
   // init options len
   coap_incomingOptionsLen = MAX_COAP_OPTIONS;
   coap_outgoingOptionsLen = 0;

   // take ownership over the received packet
   msg->owner                = COMPONENT_OPENCOAP;
   
   //=== step 1. parse the packet
   
   // parse the CoAP header and remove from packet
   index = 0;
   coap_header.Ver           = (msg->payload[index] & 0xc0) >> 6;
   coap_header.T             = (coap_type_t)((msg->payload[index] & 0x30) >> 4);
   coap_header.TKL           = (msg->payload[index] & 0x0f);
   index++;
   coap_header.Code          = (coap_code_t)(msg->payload[index]);
   index++;
   coap_header.messageID     = msg->payload[index]*256+msg->payload[index+1];
   index+=2;
   
   // reject unsupported header
   if (coap_header.Ver!=COAP_VERSION || coap_header.TKL>COAP_MAX_TKL) {
      openserial_printError(
         COMPONENT_OPENCOAP,ERR_WRONG_TRAN_PROTOCOL,
         (errorparameter_t)0,
         (errorparameter_t)coap_header.Ver
      );
      openqueue_freePacketBuffer(msg);
      return;
   }
   
   // record the token
   memcpy(&coap_header.token[0], &msg->payload[index], coap_header.TKL);
   index += coap_header.TKL;

   // remove the CoAP header
   packetfunctions_tossHeader(msg,index);
    
   // parse options and toss header
   index = opencoap_options_parse(&msg->payload[0], msg->length, coap_incomingOptions, &coap_incomingOptionsLen);

   // toss options
   packetfunctions_tossHeader(msg,index);

   // process handled options

   //== Object Security Option
   objectSecurity = opencoap_find_object_security_option(coap_incomingOptions, coap_incomingOptionsLen);
   if (objectSecurity) {
       if ((objectSecurity->length == 0 && msg->length == 0) ||
               (objectSecurity->length != 0 && msg->length != 0)) {
            // malformated object security message
            return;
       }
       
       if (objectSecurity->length == 0) {
           index = openoscoap_parse_compressed_COSE(&msg->payload[0], 
                   msg->length, 
                   &rcvdSequenceNumber,
                   &rcvdKid,
                   &rcvdKidLen);
           if (index == 0) {
               return;
           }
           packetfunctions_tossHeader(msg, index);
       }
       else {
           index = openoscoap_parse_compressed_COSE(objectSecurity->pValue,
                   objectSecurity->length,
                   &rcvdSequenceNumber,
                   &rcvdKid,
                   &rcvdKidLen);
           
           if (index == 0) {
               return;
           }
           objectSecurity->length -= index;
           objectSecurity->pValue += index;
       }
   }

   //=== step 2. find the resource to handle the packet
   
   // find the resource this applies to
   found = FALSE;
   
   if (
         coap_header.Code>=COAP_CODE_REQ_GET &&
         coap_header.Code<=COAP_CODE_REQ_DELETE
      ) {
      // this is a request: target resource is indicated as COAP_OPTION_LOCATIONPATH option(s)
      // find the resource which matches
      
      // start with the first resource in the linked list
      temp_desc = opencoap_vars.resources;
      
      // iterate until matching resource found, or no match
      while (found==FALSE) {
         if (
               coap_incomingOptions[0].type==COAP_OPTION_NUM_URIPATH    &&
               coap_incomingOptions[1].type==COAP_OPTION_NUM_URIPATH    &&
               temp_desc->path0len>0                                   &&
               temp_desc->path0val!=NULL                               &&
               temp_desc->path1len>0                                   &&
               temp_desc->path1val!=NULL
            ) {
            // resource has a path of form path0/path1
               
            if (
                  coap_incomingOptions[0].length==temp_desc->path0len                               &&
                  memcmp(coap_incomingOptions[0].pValue,temp_desc->path0val,temp_desc->path0len)==0 &&
                  coap_incomingOptions[1].length==temp_desc->path1len                               &&
                  memcmp(coap_incomingOptions[1].pValue,temp_desc->path1val,temp_desc->path1len)==0
               ) {
               found = TRUE;
            };
         
         } else if (
               coap_incomingOptions[0].type==COAP_OPTION_NUM_URIPATH    &&
               temp_desc->path0len>0                            &&
               temp_desc->path0val!=NULL
            ) {
            // resource has a path of form path0
               
            if (
                  coap_incomingOptions[0].length==temp_desc->path0len                               &&
                  memcmp(coap_incomingOptions[0].pValue,temp_desc->path0val,temp_desc->path0len)==0
               ) {
               found = TRUE;
            };
         };
         
         // iterate to next resource, if not found
         if (found==FALSE) {
            if (temp_desc->next!=NULL) {
               temp_desc = temp_desc->next;
            } else {
               break;
            }
         }
      }
   
   } else {
      // this is a response: target resource is indicated by token, and message ID
      // if an ack for a confirmable message, or a reset
      // find the resource which matches
      
      // start with the first resource in the linked list
      temp_desc = opencoap_vars.resources;
      
      // iterate until matching resource found, or no match
      while (found==FALSE) {
         
         if (
                coap_header.TKL==temp_desc->last_request.TKL                                       &&
                memcmp(&coap_header.token[0],&temp_desc->last_request.token[0],coap_header.TKL)==0
            ) {
                
            if (coap_header.T==COAP_TYPE_ACK || coap_header.T==COAP_TYPE_RES) {
                if (coap_header.messageID==temp_desc->last_request.messageID) {
                    found=TRUE;
                }
            } else {
                found=TRUE;
            }
            
            // resource found, verify if it needs to be decrypted
            if (found==TRUE && temp_desc->callbackRx!=NULL) {
                if (temp_desc->securityContext != NULL) {
                    coap_incomingOptionsLen = MAX_COAP_OPTIONS;
                    decStatus = openoscoap_unprotect_message(temp_desc->securityContext,
                            coap_header.Ver,
                            coap_header.Code,
                            coap_incomingOptions,
                            &coap_incomingOptionsLen,
                            msg,
                            temp_desc->last_request.oscoapSeqNum
                    );

                    if (decStatus != E_SUCCESS) {
                        return;
                    }
                }

               temp_desc->callbackRx(msg,&coap_header,&coap_incomingOptions[0], NULL, NULL);
            }
         }
         
         // iterate to next resource, if not found
         if (found==FALSE) {
            if (temp_desc->next!=NULL) {
               temp_desc = temp_desc->next;
            } else {
               break;
            }
         }
      };
      
      // free the received packet
      openqueue_freePacketBuffer(msg);
      
      // stop here: will will not respond to a response
      return;
   }
   
   //=== step 3. ask the resource to prepare response
   
   if (found==TRUE) {
      
      // call the resource's callback
      outcome = temp_desc->callbackRx(msg,&coap_header,&coap_incomingOptions[0], coap_outgoingOptions, &coap_outgoingOptionsLen);

   } else {
      // reset packet payload (DO NOT DELETE, we will reuse same buffer for response)
      msg->payload                     = &(msg->packet[127]);
      msg->length                      = 0;
      // set the CoAP header
      coap_header.TKL                  = 0;
      coap_header.Code                 = COAP_CODE_RESP_NOTFOUND;
   }
   
   if (outcome==E_FAIL) {
      // reset packet payload (DO NOT DELETE, we will reuse same buffer for response)
      msg->payload                     = &(msg->packet[127]);
      msg->length                      = 0;
      // set the CoAP header
      coap_header.TKL                  = 0;
      coap_header.Code                 = COAP_CODE_RESP_METHODNOTALLOWED;
   }

   if (coap_header.T == COAP_TYPE_CON) {
       response_type = COAP_TYPE_ACK;
   } else {
       response_type = COAP_TYPE_NON;
   }

   //=== step 4. add the payload marker and encode options

   if (msg->length > 0 ) { // contains payload, add payload marker
      packetfunctions_reserveHeaderSize(msg,1);
      msg->payload[0] = COAP_PAYLOAD_MARKER;
   }
      
   // once header is reserved, encode the options to the openqueue payload buffer
   opencoap_options_encode(msg, coap_outgoingOptions, coap_outgoingOptionsLen, COAP_OPTION_CLASS_ALL);

   //=== step 5. send that packet back
   
   // fill in packet metadata
   if (found==TRUE) {
      msg->creator                     = temp_desc->componentID;
   } else {
      msg->creator                     = COMPONENT_OPENCOAP;
   }
   msg->l4_protocol                    = IANA_UDP;
   temp_l4_destination_port            = msg->l4_destination_port;
   msg->l4_destination_port            = msg->l4_sourcePortORicmpv6Type;
   msg->l4_sourcePortORicmpv6Type      = temp_l4_destination_port;
   
   // set destination address as the current source
   msg->l3_destinationAdd.type         = ADDR_128B;
   memcpy(&msg->l3_destinationAdd.addr_128b[0],&msg->l3_sourceAdd.addr_128b[0],LENGTH_ADDR128b);
   
   // fill in CoAP header
   packetfunctions_reserveHeaderSize(msg,4+coap_header.TKL);
   msg->payload[0]                  = (COAP_VERSION    << 6) |
                                      (response_type   << 4) |
                                      (coap_header.TKL << 0);
   msg->payload[1]                  = coap_header.Code;
   msg->payload[2]                  = coap_header.messageID/256;
   msg->payload[3]                  = coap_header.messageID%256;
   memcpy(&msg->payload[4], &coap_header.token[0], coap_header.TKL);
   
   if ((openudp_send(msg))==E_FAIL) {
      openqueue_freePacketBuffer(msg);
   }
}

/**
\brief Indicates that the CoAP response has been sent.

\param[in] msg A pointer to the message which was sent.
\param[in] error The outcome of the send function.
*/
void opencoap_sendDone(OpenQueueEntry_t* msg, owerror_t error) {
   coap_resource_desc_t* temp_resource;
   
   // take ownership over that packet
   msg->owner = COMPONENT_OPENCOAP;
   
   // indicate sendDone to creator of that packet
   //=== mine
   if (msg->creator==COMPONENT_OPENCOAP) {
      openqueue_freePacketBuffer(msg);
      return;
   }
   //=== someone else's
   temp_resource = opencoap_vars.resources;
   while (temp_resource!=NULL) {
      if (
         temp_resource->componentID==msg->creator &&
         temp_resource->callbackSendDone!=NULL
         ) {
         temp_resource->callbackSendDone(msg,error);
         return;
      }
      temp_resource = temp_resource->next;
   }
   
   // if you get here, no valid creator was found
   
   openserial_printError(
      COMPONENT_OPENCOAP,ERR_UNEXPECTED_SENDDONE,
      (errorparameter_t)0,
      (errorparameter_t)0
   );
   openqueue_freePacketBuffer(msg);
}

//===== from CoAP resources

/**
\brief Writes the links to all the resources on this mote into the message.

\param[out] msg The messge to write the links to.
\param[in] componentID The componentID calling this function.

\post After this function returns, the msg contains 
*/
void opencoap_writeLinks(OpenQueueEntry_t* msg, uint8_t componentID) {
   coap_resource_desc_t* temp_resource;
   
   // start with the first resource in the linked list
   temp_resource = opencoap_vars.resources;
   
   // iterate through all resources
   while (temp_resource!=NULL) {
      
      if (  
            (temp_resource->discoverable==TRUE) &&
            (
               ((componentID==COMPONENT_CWELLKNOWN) && (temp_resource->path1len==0))
               || 
               ((componentID==temp_resource->componentID) && (temp_resource->path1len!=0))
            )
         ) {
          
         // write ending '>'
         packetfunctions_reserveHeaderSize(msg,1);
         msg->payload[0] = '>';
         
         // write path1
         if (temp_resource->path1len>0) {
            packetfunctions_reserveHeaderSize(msg,temp_resource->path1len);
            memcpy(&msg->payload[0],temp_resource->path1val,temp_resource->path1len);
            packetfunctions_reserveHeaderSize(msg,1);
            msg->payload[0] = '/';
         }
         
         // write path0
         packetfunctions_reserveHeaderSize(msg,temp_resource->path0len);
         memcpy(msg->payload,temp_resource->path0val,temp_resource->path0len);
         packetfunctions_reserveHeaderSize(msg,2);
         msg->payload[1] = '/';
         
         // write opening '>'
         msg->payload[0] = '<';
         
         // write separator between links
         if (temp_resource->next!=NULL) {
            packetfunctions_reserveHeaderSize(msg,1);
            msg->payload[0] = ',';
         }
      }
      // iterate to next resource
      temp_resource = temp_resource->next;
   }
}

/**
\brief Register a new CoAP resource.

This function is called by a CoAP resource when it starts, allowing it to
receive data sent to that resource.

Registration consists in adding a new resource at the end of the linked list
of resources.

\param[in] desc The description of the CoAP resource.
*/
void opencoap_register(coap_resource_desc_t* desc) {
   coap_resource_desc_t* last_elem;
   
   // since this CoAP resource will be at the end of the list, its next element
   // should point to NULL, indicating the end of the linked list.
   desc->next = NULL;
   
   // if this is the first resource, simply have resources point to it
   if (opencoap_vars.resources==NULL) {
      opencoap_vars.resources = desc;
      return;
   }
   
   // if not, add to the end of the resource linked list
   last_elem = opencoap_vars.resources;
   while (last_elem->next!=NULL) {
      last_elem = last_elem->next;
   }
   last_elem->next = desc;
}

/**
\brief Send a CoAP request.

This function is called by a CoAP resource when it wants to send some data.
This function is NOT called for a response.

\param[in] msg The message to be sent. This messages should not contain the
   CoAP header.
\param[in] type The CoAP type of the message.
\param[in] code The CoAP code of the message.
\param[in] TKL  The Token Length of the message, sanitized to a max of COAP_MAX_TKL (8).
\param[in] options An array of sorted CoAP options.
\param[in] optionsLen The length of the options array.
\param[out] descSender A pointer to the description of the calling CoAP
   resource.

\post After returning, this function will have written the messageID and TOKEN
   used in the descSender parameter.

\return The outcome of sending the packet.
*/
owerror_t opencoap_send(
      OpenQueueEntry_t*      msg,
      coap_type_t            type,
      coap_code_t            code,
      uint8_t                TKL,
      coap_option_iht*       options,
      uint8_t                optionsLen,
      coap_resource_desc_t*  descSender
   ) {
   uint16_t token;
   uint8_t tokenPos=0;
   coap_header_iht* request;
   owerror_t ret;
   coap_option_class_t class;

   class = COAP_OPTION_CLASS_ALL;

   // increment the (global) messageID
   if (opencoap_vars.messageID++ == 0xffff) {
      opencoap_vars.messageID = 0;
   }
   
   // take ownership over the packet
   msg->owner                       = COMPONENT_OPENCOAP;
   
   // fill in packet metadata
   msg->l4_sourcePortORicmpv6Type   = WKP_UDP_COAP;
   
   // update the last_request header
   request                          = &descSender->last_request;
   request->T                       = type;
   request->Code                    = code;
   request->messageID               = opencoap_vars.messageID;
   request->TKL                     = TKL<COAP_MAX_TKL ? TKL : COAP_MAX_TKL;
   
   while (tokenPos<request->TKL) {
       token = openrandom_get16b();
       memcpy(&request->token[tokenPos],&token,2);
       tokenPos+=2;
   }

   if (descSender->securityContext != NULL) { // security activated for the resource
      // get new sequence number and save it
      request->oscoapSeqNum = openoscoap_get_sequence_number(descSender->securityContext);
      // protect the message in the openqueue buffer
      ret = openoscoap_protect_message(
              descSender->securityContext,
              COAP_VERSION,
              code,
              options,
              optionsLen,
              msg,
              request->oscoapSeqNum);

      if (ret != E_SUCCESS) {
         return E_FAIL;
      }
      class = COAP_OPTION_CLASS_U;
   }
      
   // once header is reserved, encode the options to the openqueue payload buffer
   opencoap_options_encode(msg, 
           options, 
           optionsLen, 
           class);

   // pre-pend CoAP header (version,type,TKL,code,messageID,Token)
   packetfunctions_reserveHeaderSize(msg,4+request->TKL);
   msg->payload[0]                  = (COAP_VERSION   << 6) |
                                      (type           << 4) |
                                      (request->TKL   << 0);
   msg->payload[1]                  = code;
   msg->payload[2]                  = (request->messageID>>8) & 0xff;
   msg->payload[3]                  = (request->messageID>>0) & 0xff;

   memcpy(&msg->payload[4],&token,request->TKL);
   
   return openudp_send(msg);
}

/**
\brief Lookup the OSCOAP class for a given option.

This function is called to resolve the OSCOAP class of the passed option.
CLASS_E options get encrypted, CLASS_I options are integrity protected,
and CLASS_U options are unprotected by OSCOAP, if security is activated.

\param[in] type The CoAP option type that needs to be resolved.
*/
coap_option_class_t opencoap_get_option_class(coap_option_t type) {
    switch(type) {
        // class E options
        case COAP_OPTION_NUM_IFMATCH:
        case COAP_OPTION_NUM_ETAG:
        case COAP_OPTION_NUM_IFNONEMATCH:
        case COAP_OPTION_NUM_LOCATIONPATH:
        case COAP_OPTION_NUM_URIPATH:
        case COAP_OPTION_NUM_CONTENTFORMAT:
        case COAP_OPTION_NUM_MAXAGE:
        case COAP_OPTION_NUM_URIQUERY:
        case COAP_OPTION_NUM_ACCEPT:
        case COAP_OPTION_NUM_LOCATIONQUERY:
            return COAP_OPTION_CLASS_E;
        // class I options none supported
        
        //class U options
        case COAP_OPTION_NUM_URIHOST:
        case COAP_OPTION_NUM_URIPORT:
        case COAP_OPTION_NUM_PROXYURI:
        case COAP_OPTION_NUM_PROXYSCHEME:
        case COAP_OPTION_NUM_OBJECTSECURITY:
            return COAP_OPTION_CLASS_U;
        default:
            return COAP_OPTION_CLASS_U;    
    }
}

owerror_t opencoap_options_encode(
        OpenQueueEntry_t*       msg,
        coap_option_iht*        options,
        uint8_t                 optionsLen,
        coap_option_class_t     class
        ) {

    uint8_t i;
    uint8_t ii;
    uint32_t delta;
    uint8_t optionDelta;
    uint8_t optionDeltaExt[2];
    uint8_t optionDeltaExtLen;
    uint8_t optionLength;
    uint8_t optionLengthExt[2];
    uint8_t optionLengthExtLen;
    coap_option_t previousOptionNum;

    // encode options in reversed order
    if (options != NULL && optionsLen != 0) {
        for (i = optionsLen ; i-- > 0 ; ) {
            // skip option if inappropriate class
            if (class != opencoap_get_option_class(options[i].type) && 
                class != COAP_OPTION_CLASS_ALL) {
                continue;
            }
            
            // loop to find the previous option to which delta should be calculated
            previousOptionNum = COAP_OPTION_NONE;
            for (ii = i ; ii-- > 0 ; ) {
                if (class != opencoap_get_option_class(options[ii].type) && 
                    class != COAP_OPTION_CLASS_ALL) {
                    continue;
                }
                else {
                    previousOptionNum = options[ii].type;
                    break;
                }
            }

            if (previousOptionNum > options[i].type) {
                return E_FAIL; // we require the options to be sorted
            }
            delta = options[i].type - previousOptionNum;

            if (delta <= 12) {
                optionDelta = (uint8_t) delta;
                optionDeltaExtLen = 0; 
            }
            else if (delta <= 0xff + 13) {
                optionDelta = 13;
                optionDeltaExt[0] = (uint8_t) delta - 13;
                optionDeltaExtLen = 1;
            }
            else if (delta <= 0xffff + 269) {
                optionDelta = 14;
                packetfunctions_htons((uint16_t) delta - 269, optionDeltaExt); 
                optionDeltaExtLen = 2;
            }
            else {
                return E_FAIL;
            }

            if (options[i].length <= 12) {
                optionLength = options[i].length;
                optionLengthExtLen = 0;
            }
            else { 
                // we do not support fragmentation so option length cannot be larger
                // than 0xff. therefore, we default to the case where optionLength = 13.
                // see RFC7252 Section 3.1 for more details.
                optionLength = 13;
                optionLengthExt[0] = options[i].length - 13;
                optionLengthExtLen = 1;
            }
            
            // write to packet in reversed order
            packetfunctions_reserveHeaderSize(msg, options[i].length);
            memcpy(&msg->payload[0], options[i].pValue, options[i].length);
            
            packetfunctions_reserveHeaderSize(msg, optionLengthExtLen);
            memcpy(&msg->payload[0], optionLengthExt, optionLengthExtLen);

            packetfunctions_reserveHeaderSize(msg, optionDeltaExtLen);
            memcpy(&msg->payload[0], optionDeltaExt, optionDeltaExtLen);

            packetfunctions_reserveHeaderSize(msg, 1);
            msg->payload[0] = (optionDelta << 4) | optionLength;
        }
    }
    return E_SUCCESS;
}

coap_option_iht* opencoap_find_object_security_option(coap_option_iht* array, uint8_t arrayLen) {
    uint8_t i;

    if (array == NULL || arrayLen == 0) {
        return NULL;
    }

    for (i = 0; i < arrayLen; i++) {
        if (array[i].type == COAP_OPTION_NUM_OBJECTSECURITY) {
            return &array[i];
        }
    }
    return NULL;
}

//=========================== private =========================================

uint8_t opencoap_options_parse(
        uint8_t*                buffer,
        uint8_t                 bufferLen,
        coap_option_iht*        options,
        uint8_t*                optionsLen
        ) {

    uint8_t index;
    uint8_t i;
    coap_option_t lastOption;
    coap_option_t optionDelta;
    uint8_t optionLength;
    uint8_t numOptions;

    index = 0;
    numOptions = 0;

    // initialize the coap_incomingOptions
    for (i=0;i<*optionsLen;i++) {
        options[i].type = COAP_OPTION_NONE;
        options[i].length = 0;
        options[i].pValue = NULL;
    }
   
    lastOption = COAP_OPTION_NONE;
    for (i = 0; i < *optionsLen; i++) {
      
        // detect when done parsing options
        if (buffer[index]==COAP_PAYLOAD_MARKER) {
            // found the payload marker, done parsing options.
            index++; // skip marker and stop parsing options
            break;
        }
        if (bufferLen<=index) {
             // end of message, no payload
            break;
        }

        optionDelta = ((buffer[index] & 0xf0) >> 4);
        optionLength = (buffer[index] & 0x0f);

        index++;

        if (optionDelta <= 12) {
        }
        else if (optionDelta == 13) {
            optionDelta = buffer[index] + 13;
            index++;
        }
        else if (optionDelta == 14) {
            optionDelta = (coap_option_t) (packetfunctions_ntohs(&buffer[index]) + 269);
            index += 2;
        }
        else {
            break;
        }

        if (optionLength <= 12) {

        }
        else if (optionLength == 13) {
            optionLength = buffer[index] + 13;
            index++;
        }
        else {
            // case 14 not supported
            break;
        }

        if (bufferLen <= index) {
            break;
        }
         
        // create new option
        options[i].type = lastOption + optionDelta;
        options[i].length = optionLength;
        if (optionLength) {
            options[i].pValue = &(buffer[index]);
        }
        index += optionLength;
        lastOption = options[i].type;
        numOptions++;
    }
    *optionsLen = numOptions;
    return index;
}


