% calcuation of the FCS
% payload: abcdef[ENTER] - 61 62 63 64 65 66 0A

%      |--------------------------MHR-----------------------|---------------MSDU--------------------...
MPDU = ['61'; '88'; 'CF'; 'AD'; 'DE'; 'EE'; 'BE'; 'EF'; 'BE'; '61'; '62'; '63'; '64'; '65'; '66'; '0A']; 

% bit reversal for one byte
mat = [0 0 0 0 0 0 0 1;
       0 0 0 0 0 0 1 0;
       0 0 0 0 0 1 0 0;
       0 0 0 0 1 0 0 0;
       0 0 0 1 0 0 0 0;
       0 0 1 0 0 0 0 0;
       0 1 0 0 0 0 0 0;
       1 0 0 0 0 0 0 0;
       ];

binString = [];

% convert all hex data into binary 
% make one long binary string 
for i=1:size(MPDU, 1)
    shortString = mat *  hexToBinaryVector(MPDU(i, :), 8)';
    binString = [binString shortString'];
end

% multiplication with x^16
leftShift = zeros(16, 1);
binString = [binString leftShift'];

% generator polynomial 
div = [1 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 1];

% polynomial devision - remainder: r
[a, r] = gfdeconv(fliplr(binString), fliplr(div), 2);

r

% conversion to hex-numbers
convertToHex = [128 64 32 16 8 4 2 1];

dec2hex(convertToHex * r(9:end)')
dec2hex(convertToHex * r(1:8)')

