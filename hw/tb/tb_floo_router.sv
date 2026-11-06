// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Michael Rogenmoser <michaero@iis.ee.ethz.ch>

`include "floo_noc/typedef.svh"

  // TODO MICHAERO: gen stimuli
  //   - for each input VC generate packets of flits in a queue with random destination Ports (IDs), excluding the source port
  //   - collect these packets for each destination port (ID) and VC
  //   - for each output VC collect all incoming packets of flits
  //   - match the collected flits to ensure all packets are routed to their intended destination without errors or injected/missing flits


module tb_floo_router;

  import floo_pkg::*;

  localparam time CyclTime = 10ns;
  localparam time ApplTime = 2ns;
  localparam time TestTime = 8ns;

  task automatic cycle_start();
    #ApplTime;
  endtask : cycle_start

  task automatic cycle_end();
    #TestTime;
  endtask : cycle_end

  task automatic started_cycle_end();
    #(TestTime-ApplTime);
  endtask : started_cycle_end

  localparam int unsigned NumTestPacketsPerChannel = 100;

  localparam int unsigned NumPorts = 5;
  localparam int unsigned NumVirtChannels = 6;
  localparam int unsigned IdWidth = $clog2(NumPorts);
  localparam int unsigned FlitWidth = 123;
  localparam int unsigned MaxPacketLength = 32;

  typedef logic [FlitWidth-1:0] payload_t;
  typedef logic [IdWidth-1:0] id_t;

  `FLOO_TYPEDEF_HDR_T(hdr_t, id_t, id_t, logic, logic)  //crea typedef struct hdr_t (la struttura dell'header)
  `FLOO_TYPEDEF_GENERIC_FLIT_T(req, hdr_t, payload_t)  //nome, tipo, tipo
  //crea il tipo floo_req_generic_flit_t cioè il tipo del flit

  logic clk, rst_n;

  clk_rst_gen #(
    .ClkPeriod    ( CyclTime ),
    .RstClkCycles ( 5        )
  ) i_clk_gen (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );


  /************************
   *  Stimuli generation  *
   ************************/

  class rand_data_t;
    bit [IdWidth-1:0] id;  // source flit
    rand logic [FlitWidth-1:0] data;  //payload

    function new (bit [IdWidth-1:0] in_id);  
      this.id = in_id;
    endfunction

    constraint data_mod_id_c {  //riesco in questo modo ad associare ogni playload a una porta, serve per scovare errori
      data % NumPorts == id;
    }
  endclass

  class stimuli_t;
    bit [IdWidth-1:0]        source; //sorgente flit
    rand int                 len;   //lunghezza del pacchetto
    rand bit [IdWidth-1:0]   id;  //destinazione flit

    function new (bit [IdWidth-1:0] in_source); //tramite assegnazione determino porta sorgente
      this.source = in_source;
    endfunction

    constraint id_in_range_c {    // porta destinazione non può essere uguale a sorgente
      id < NumPorts;
      id != source;
    }

    constraint len_is_reasonable_c {  //il pacchetto può avere massimo 32 flits
      len <= 32;
      len > 0;
    }

  endclass

  //                         Source Port   Virtual Channel
  floo_req_generic_flit_t  stimuli_queue[NumPorts][NumVirtChannels][$]; //dinamiche, cioè possono cambiare dimensione durante simulazione!

  //                         Destination   Virtual Channel      Source Port
  floo_req_generic_flit_t  golden_queue [NumPorts][NumVirtChannels][NumPorts][$];

  function automatic void generate_stimuli();
    for (int port = 0; port < NumPorts; port++) begin
      for (int virt_channel = 0; virt_channel < NumVirtChannels; virt_channel++) begin
        // TODO MICHAERO: ensure each output gets at least one packet from input for testbench termination

        for (int i = 0; i < NumTestPacketsPerChannel; i++) begin  //100 pacchetti per vc e per porta
          automatic stimuli_t stimuli = new(port);  //stimuli.source=port

          // Active Constraints  //di default sono già attivate, quindi ad (1) (0) per disattivarle
          stimuli.id_in_range_c.constraint_mode(1);
          stimuli.len_is_reasonable_c.constraint_mode(1);

          // Randomize
          if (stimuli.randomize()) begin  //stimuli.id stimuli.len hanno valore casuale
            
            //NOTA BENE: tutti i flit del pacchetto avranno stessa porta destinazione (per quella porta sorgente e quel vc associato)
            
            //creazione del pacchetto
            //stimuli ha parametro source=port id e len casuali
            for (int j = 0; j < stimuli.len; j++) begin  //dimensione del pacchetto
              //creo i flits:
              
              automatic rand_data_t rand_data = new(port);  //rand_data.id=port;
              rand_data.data_mod_id_c.constraint_mode(1);
              
              if (rand_data.randomize()) begin  //rand_data.data ha valore casuale
                //rand_data ha id=port e data valore casuale
                
                automatic floo_req_generic_flit_t next_flit = '0; //creato un nuovo flit
                
                //definizione del payload del flit
                next_flit.payload = rand_data.data;
                
                //definizione dell'header del flit:
                next_flit.hdr.src_id = port; //porta da cui entra il flit
                next_flit.hdr.dst_id = stimuli.id; //porta da cui esce il flit
                next_flit.hdr.last = j == stimuli.len-1;  //bit che indica ultimo flit (quando si arriva a fine ciclo viene impostato a uno)
                

                stimuli_queue[port][virt_channel].push_back(next_flit);  //alla porta x e al virtual channel y ci sta una FIFO
                
                golden_queue[port][virt_channel][stimuli.id].push_back(next_flit);  //devo segnare anche la destinazione
                //i flits di un pacchetto hanno la stessa destinazione!!!

                // $display("%x from: %d to: %d vc: %d, last: %d",next_flit.data, port, next_flit.id, virt_channel, next_flit.last);
              end else begin
                $error("Could not randomize.");
              end
            end
          end else begin
            $error("Could not randomize.");
          end
        end
      end
    end
  endfunction

  // Apply Stimuli

  logic       [NumPorts-1:0][NumVirtChannels-1:0] pre_valid_in, pre_ready_in;  //valid e ready
  
  floo_req_generic_flit_t [NumPorts-1:0][NumVirtChannels-1:0] pre_data_in;  //dati
  

  //la task tratta un solo flit, non un intero pacchetto, va ripetuta

  task automatic apply_stimuli(int unsigned port, int unsigned virt_channel);   //passo alla task la porta e il vc che voglio trattare
    
    automatic floo_req_generic_flit_t stimuli;  //da non confondere con oggetto stimuli della classe stimuli_t usata nella function stimuli gen

    // pre_valid_in[port][virt_channel] = 1'b0;

    wait (stimuli_queue[port][virt_channel].size() != '0);  //aspetta che la queue abbia almeno un elemento
    
    fork //processi in parallelo
  
     //processo1 
      begin : apply_valid_data
        //2 ns  --vedere sezione finale: testbench --ma solo all'inizio
        stimuli = stimuli_queue[port][virt_channel].pop_front(); //estrae il flit dalla queue
        pre_data_in[port][virt_channel] = stimuli;  //lo dà al vettore di dati
        pre_valid_in[port][virt_channel] = 1'b1;  //alza il valore di valid a 1
      end
      //processo2
      begin
        started_cycle_end();  // ritardo di 6 ns --> siamo a 8 ns
        while (!pre_ready_in[port][virt_channel]) begin  //finché non ricevo segnale di ready aspetto
          //quando lo ricevo
          @(posedge clk)  //rising edge del clock  --qua avviene il trasferimento e valid deve essere 1
          cycle_end(); //aspetto 8 ns --> siamo a 16 ns
        end
      end
    join  //prosegue solo quando sia processo1 che processo2 sono terminati

    //siamo a 16 ns al primo ciclo
    @(posedge clk) //al rising edge //20 ns
    cycle_start(); //aspetto 2 ns in modo da ripristinare i vincoli temporali: rising edge-- 2ns ---valid---8ns ready---
    pre_valid_in[port][virt_channel] = 1'b0;  //pongo a 0 il segnale di valid

    ///boh qualcosa non mi convince, non capisco il senso di questi vincoli temporali, da rivedere

  endtask

  /***********************
   *  Device Under Test  *
   ***********************/

  logic       [NumPorts-1:0][NumVirtChannels-1:0] delayed_valid_in;
  logic       [NumPorts-1:0][NumVirtChannels-1:0] delayed_ready_in;
  floo_req_generic_flit_t [NumPorts-1:0][NumVirtChannels-1:0] delayed_data_in;

  //AGGIUNTA DI RITARDO IN TRASMISSIONE
  //ho uno stream delay per ciascun virtual channel di ogni porta, quindi 30 in totale
  for (genvar port = 0; port < NumPorts; port++) begin : gen_in_delay
    for (genvar vc = 0; vc < NumVirtChannels; vc++) begin : gen_in_vc_delay
      cc_stream_delay #(
        .StallRandom ( 1'b1        ),  //introduce 0 o 1 clock cycle di ritardo
        .FixedDelay  ( 1           ),
        .payload_t   ( floo_req_generic_flit_t ),
        .Seed        ( '0          )
      ) i_in_delay (
        .clk_i    (clk),
        .rst_ni   (rst_n),

        .payload_i( pre_data_in [port][vc] ),
        .ready_o  ( pre_ready_in[port][vc] ),   //è un output, lo stesso che viene controllato nella task apply_stimuli()
        .valid_i  ( pre_valid_in[port][vc] ),

        .payload_o( delayed_data_in [port][vc] ), 
        .ready_i  ( delayed_ready_in[port][vc] ),
        .valid_o  ( delayed_valid_in[port][vc] )
      );
    end
  end

  //FUNZIONAMENTO STREAM DELAY -- utilizzato per introdurre ritardi e valutare come risponde il router agli stalli
  //cc_stream_delay ritarda solo i segnali di valid e ready
  //si basa su fsm a tre stati idle valid e ready
  //in idle valid_o e ready_o sono zero
  //quando valid_i diventa 1 si passa allo stato valid e ci si resta per il conteggio imposto da stall random o fixed
  //terminato il conteggio si va a ready, in cui valid_o=1 e ready_o=ready_i, quando ready_i diventa zero si torna nell'idle
  //-- controllare meglio LFSR

  //ARBITER

  logic       [NumPorts-1:0][NumVirtChannels-1:0] valid_in,   valid_out;  
  logic       [NumPorts-1:0][NumVirtChannels-1:0] ready_in,   ready_out;
  floo_req_generic_flit_t [NumPorts-1:0]                      data_in,  data_out; //in and out in riferimento al router, sono vettori di flit
  //data in è ciò che esce da arbitro ed entra nel router
  //data out è ciò che esce dal router
  //delayed_data_in è una matrice, contiene 30 flits

  //un arbitro per ogni porta

  for (genvar port = 0; port < NumPorts; port++) begin : gen_in_vc_arb
    floo_vc_arbiter #(
      .NumVirtChannels ( NumVirtChannels ),
      .flit_t          ( floo_req_generic_flit_t )
    ) i_vc_arbiter (
      .clk_i   ( clk   ),
      .rst_ni  ( rst_n ),

      //sto prendendo tutti i flit (6) per ogni porta (quindi non un flit per vc)
      //tutti i segnali valid per ogni porta (quindi 6)
      .valid_i ( delayed_valid_in[port] ),  /// esempio 110100 (6 bit)
      .ready_o ( delayed_ready_in[port] ),  //stessa dimensione di ready_in, ma solo un bit pari a 1, perché ne fa passare solo uno!!! attiva handshake
      .data_i  ( delayed_data_in [port] ),  //6 flit

      .valid_o ( valid_in[port] ),   //stessa dimensione del delayed_valid_in, ma solo un suo bit è 1!!!  esempio: 001000, perché ne ha fatto passare solo uno!
      .ready_i ( ready_in[port] ),  //ha 6 bit, ma più bit possono essere 1
      .data_o  ( data_in [port] ), //contiene un solo flit!!!!
      .credit_i( '0             )
    );
  end

//arbitro non ha registri, è solo uno smistatore, dà il ready_o=1 ai vc per cui ha decretato il passaggio



  // -------- ROUTER ------- DUT
  //solo source routing, non usa xy né id-based routing
  //capire meglio il ruolo di idwidth
  //niente meccanismo dei crediti, forse perché il corner case per maggior backpressure non usa i crediti??
  //no operazioni collettive/riduzioni  --capirle meglio

  //grazie ad arbitro arriva un flusso di dati a porta (un flit)
  floo_router #(
    .NumRoutes        ( NumPorts                ),
    .NumVirtChannels  ( NumVirtChannels         ),
    .flit_t           ( floo_req_generic_flit_t ),
    .InFifoDepth      ( 4                       ),  
    .RouteAlgo        ( SourceRouting           ),
    .IdWidth          ( IdWidth                 )
  ) i_dut (
    .clk_i         ( clk   ),
    .rst_ni        ( rst_n ),
    .test_enable_i ( '0 ),

    .xy_id_i       ( '0 ), // Unused for `SourceRouting`
    .id_route_map_i( '0 ), // Unused for `SourceRouting`

    // ho 5 porte, quindi sicuramente devo ricevere 5 dati (flits) con relativi segnali di valid
    //e altrettanti devono mandarne fuori
    //valid_in,out ready_in,out sono matrici 5x6 (ports x vc) data_in,out sono vettori di flits da 5 elementi (ports)
    .valid_i       ( valid_in  ),  // 5 vettori da 6 bits
    .ready_o       ( ready_in  ),  // 5 vettori da 6 bits
    .data_i        ( data_in   ), //5 flits -- 1 per porta
    .credit_o      (           ),

    .valid_o       ( valid_out ),
    .ready_i       ( ready_out ),
    .data_o        ( data_out  ),
    .credit_i      ( '0        ),
    .offload_req_o (           ),  
    .offload_rsp_i ( '0        )
  );


  //FALL REGISTER:
  //nella fifo salva il flit (ha spazio solo per un flit)
  //se la sua fifo è vuota dà segnale di ready_o=1, altrimenti ready_o=0;
  //se la fifo è vuota e a valle c'è disponibilità cioè read_i=1, il dato passa direttamente (FALL_THROUGH) con valid_o=1;
  //in caso contrario viene salvato nella fifo, con valid_o=1
  
  logic       [NumPorts-1:0][NumVirtChannels-1:0] fall_valid_out;
  logic       [NumPorts-1:0][NumVirtChannels-1:0] fall_ready_out;
  floo_req_generic_flit_t [NumPorts-1:0][NumVirtChannels-1:0] fall_data_out;  //matrice di flits (5x6)
  logic       [NumPorts-1:0][NumVirtChannels-1:0] delayed_valid_out;
  logic       [NumPorts-1:0][NumVirtChannels-1:0] delayed_ready_out;
  floo_req_generic_flit_t [NumPorts-1:0][NumVirtChannels-1:0] delayed_data_out;


  //FUNGE ANCHE DA MUX PER READY E VALID SIGNAL, ma ha anche altro ruolo? BOh
  // ne vengono istanziati 30 
  for (genvar port = 0; port < NumPorts; port++) begin : gen_out_delay
    for (genvar vc = 0; vc < NumVirtChannels; vc++) begin : gen_out_vc_delay

      cc_fall_through_register #(
        .data_t ( floo_req_generic_flit_t )
      ) i_fall (
        .clk_i     (clk),
        .rst_ni    (rst_n),
        .clr_i     (1'b0),

        .valid_i   ( valid_out     [port][vc] ), //1 solo bit 
        .ready_o   ( ready_out     [port][vc] ), //1 solo bit
        .data_i    ( data_out      [port] ),    //1 flit   --dat_out è vettore di flits, contenente 5 elementi

        .valid_o   ( fall_valid_out[port][vc] ),  //1 solo bit
        .ready_i   ( fall_ready_out[port][vc] ),   //1 solo bit
        .data_o    ( fall_data_out [port][vc] )   // 1 flit  
      );  //sto ricostruendo la matrice sulla base dei segnali di valid, io riuscirò ad attribuire a ciascun vc della porta il giusto dato 

      //ne vengono istanziati 30
      //introduce ritardo per testare la backpressure, ritardi in ricezione
      cc_stream_delay #(
        .StallRandom ( 1'b1        ),
        .FixedDelay  ( 1           ),
        .payload_t   ( floo_req_generic_flit_t ),
        .Seed        ( '0          )
      ) i_in_delay (
        .clk_i    (clk),
        .rst_ni   (rst_n),

        .payload_i( fall_data_out [port][vc] ),
        .ready_o  ( fall_ready_out[port][vc] ),
        .valid_i  ( fall_valid_out[port][vc] ),

        .payload_o( delayed_data_out [port][vc] ),
        .ready_i  ( delayed_ready_out[port][vc] ),
        .valid_o  ( delayed_valid_out[port][vc] )
      );
    end
  end

  /***********************
   *  Output collection  *
   ***********************/

  //                       Destination   Virtual Channel
  floo_req_generic_flit_t result_queue[NumPorts][NumVirtChannels][$];  //raccoglie i flits di ciascuna porta e rispettivo canale --la porta è quella da cui è uscito il flit

  //COLLEZIONE
  task automatic collect_result(int unsigned port, int unsigned virt_channel);  //come parametri riceve il numero di una porta e il numero di un canale

    fork
      //processo 1
      begin
        delayed_ready_out[port][virt_channel] = 1'b1;  //a valle dà disponibilità a ricevere
      end
      begin
        //processo 2 
        started_cycle_end(); //dà disponibilità a ricevere e dopo 6 ns controlla se si ha un dato valido -- boh?
        if (delayed_valid_out[port][virt_channel]) begin //a monte si ha un dato valido da mandare //ma non dovrebbe controllare anche che ready sia 1?
          result_queue[port][virt_channel].push_back(delayed_data_out[port][virt_channel]); //metto nella queue il dato
        
        end
      end
    join
    @(posedge clk);  //probabilmente qua avviene il trasferimento HANDSHAKE, quindi ready deve essere 1
    cycle_start(); //2 ns dopo il rising edge
    delayed_ready_out[port][virt_channel] = 1'b0; //abbasso il segnale di ready

  endtask

  //CHECK
  logic [NumPorts-1:0][NumVirtChannels-1:0] check_complete;

  //golden_queue[port][virt_channel][stimuli.id].push_back(next_flit); con cosa farò il check
  //porta origine, virtual channel, porta destinazione

  task automatic check_result(int unsigned port, int unsigned virt_channel);  //controllo porta e canale di DESTINAZIONE!!!
    
    logic last_active = 1'b0;  //sarebbe???
    int unsigned last_physical_bin = 0;  //sarebbe????
    
    int unsigned all_golden_size = 0;

    automatic floo_req_generic_flit_t result;
    automatic floo_req_generic_flit_t golden;

    do begin // while (all_golden_size != 0); la condizione -- il controllo continua finché i flit non sono esauriti
      wait(result_queue[port][virt_channel].size() != 0);  //aspetto finché ho almeno un flit raccolto

      // Capture the result
      
      if (result_queue[port][virt_channel].size() == 0) begin  //ma come ci arriva a sta condizione se davvero size è 0? boh il wait diventa perenne
        $error("ERROR! Result queue is empty.");
      end else begin
        result = result_queue[port][virt_channel].pop_front(); //estrae il flit e lo mette in result
      end               //ne estrae la porta sorgente   
      if (golden_queue[result.hdr.src_id][virt_channel][port].size() == 0) begin  //controlla che ci sia il flit nella golden
        $error("ERROR! Golden queue %d is empty.", result.hdr.src_id);
      end else begin
        golden = golden_queue[result.hdr.src_id][virt_channel][port].pop_front();  //estrae flit dalla golden
      end

      //IL CONFRONTO
      if (result.payload != golden.payload) begin  //confronta i payload  //il confronto tra header è implicito
        $error("ERROR! Mismatch for port %d channel %d (from %d, target port %d)",
               port, virt_channel, result.hdr.src_id, result.hdr.dst_id);
      end
      //tuttavia questo confronta solo un flit associato a porta e vc, in realtà bisogna confrontare tutti i flits definiti da port e vc presenti in results,
      

      all_golden_size = 0;
      for (int i = 0; i < NumPorts; i++) begin
        //data una porta destinazione e il suo vc, posso avere diverse porte sorgenti per i flits che dovrebbero uscire da lì
        //quindi fissati virt channel e porta sorgente, che vengono passati come parametri,
        //controllo quanti flits ho salvati nella golden queue con il comando size
        
        //size ovviamente cambia, perché durante il confronto io elimino elementi da golden_queue
        //ma se ho nella results ho meno flits che nella golden? non lo capisce e continua all'infinito? e se ho duplicazioni invece porta ad errore
        //ma solo se escono prima che la golden venga svuotata
        all_golden_size = all_golden_size + golden_queue[i][virt_channel][port].size();
      end
    end while (all_golden_size != 0);  //condizione del blocco, lo ripete finché ci sono flits

    check_complete[port][virt_channel] = 1'b1; 
  endtask

  /****************
   *  Test Bench  *
   ****************/

  //DRIVER
  task automatic run_apply(int unsigned port, int unsigned virt_channel);   //ripete all'infinito la task apply_stimuli, ma non era meglio metterci un segnale di end?
    @(posedge clk);  //solo all'inizio                                      //perché a task tratta un solo flit, ma noi vogliamo intero package
    cycle_start();  //2ns
    forever begin
      apply_stimuli(port, virt_channel);
    end
  endtask : run_apply

  //COLLECTOR
  task automatic run_collect(int unsigned port, int unsigned virt_channel);  //ripete all'infinito la task collect_result  perché anche questa task estrae solo un flit
    @(posedge clk);
    cycle_start();
    forever begin
      collect_result(port, virt_channel); 
    end
  endtask : run_collect

  //il checker non necessita di ripetizione perché autonomamente 
  initial begin
    // Initialize variables
    //ARRAY BIDIMENSIONALI DEL BLOCCO DRIVER
    pre_valid_in = '0;
    pre_data_in = '0;
    delayed_ready_out = '0;

    check_complete = '0; //matrice 5x6 di logic del blocco checker

    @(posedge rst_n)  //INIZIO quando il reset è disattivato


    //attenzione fork...join _none:
    //i processi figli vengono eseguiti solo quando il processo principale è sospeso o terminato (sospeso significa che è in attesa a causa di wait o ritardo introdotto)
    //il processo padre continua anche se i figli non hanno finito
    for (int port = 0; port < NumPorts; port++) begin
      automatic int internal_port = port;  
      //serve automatic proprio in modo che ogni processo figlio abbia la sua copia indipendente --rivedere
      //anche perché i processi figli verranno eseguiti dopo terminazione del processo principale

      fork
        for (int virt_channel = 0; virt_channel < NumVirtChannels; virt_channel++) begin
          automatic int internal_virt_channel = virt_channel;  //ogni iterazione ha la propria copia automatica
          fork
            //processi concorrenti  //ogni porta in maniera concorrente ha ogni vc che ha 3 processi attivi in maniera concorrente:  (90 processi attivi in totale)
            run_apply(internal_port, internal_virt_channel);   
            run_collect(internal_port, internal_virt_channel);
            check_result(internal_port, internal_virt_channel);
          join_none
        end
      join_none
    end
    generate_stimuli();
    while(check_complete != {NumPorts{{NumVirtChannels{1'b1}}}} ) begin  //finché check_complete non è matrice piena di 1 (5x6) ripete il ciclo while
      @(posedge clk);  //la prima volta qua il processo principale si sospende e i processi figli vengono eseguiti
    end
    $finish(0);

  end

endmodule
